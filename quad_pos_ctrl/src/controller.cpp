#include "controller.h"

void Controller::controller_loop()
{
    already_running = true;
    /*  change the frequency of controller in .launch  */
    ros::WallRate ctrl_rate(frequency);
    status_ref.header = ros::Time::now();
    status_ref.pos_d << 0.0f, 0.0f, 0.0f;
    status_ref.yaw_d = 0.0f;
    status_ref.cmd_mask = P_C_V;
    while (ros::ok())
    {
        if (ros::Time::now() - get_state().header < ros::Duration(2.0f))
        {
            pthread_mutex_lock(&ctrl_mutex);
            cmd_s tmp_ref = status_ref;
            pthread_mutex_unlock(&ctrl_mutex);
            State_s state_now = get_state();
            if (!tmp_ref.valid)
            {
                tmp_ref.pos_d << state_now.Pos(0), state_now.Pos(1), state_now.Pos(2) + 0.1f;
                Eigen::Vector3d euler;
                get_euler_from_q(euler, state_now.att_q);
                tmp_ref.yaw_d = euler(2);
            }
            /*  main function */
            one_step(tmp_ref);
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "State Timeout");
            ctrl_core.reset();
        }
        ctrl_rate.sleep();
    }
}

void Controller::one_step(const cmd_s &_ref)
{
    /*  check arm status    */
    bool arm_state = false;
    bool ofb_enable = false;
    mavros_interface.get_status(arm_state, ofb_enable);
    pthread_mutex_lock(&ctrl_mutex);
    arm_status.header = ros::Time::now();
    arm_status.armed = (arm_state && ofb_enable);
    pthread_mutex_unlock(&ctrl_mutex);

    ctrl_core.set_ref(_ref);
    bool has_armed = arm_status.armed;
    if (has_armed)
    {
        /*  calculate PID control command   */
        PID_ctrl<cmd_s, State_s>::res_s ctrl_res;
        ctrl_core.run(get_state(), ctrl_res);

        /*  PID control command -> thrust & attitude   */
        U_s U = cal_Rd_thrust(ctrl_res, _ref);

        if ((ros::Time::now() - last_ctrol_timestamp) > ros::Duration(0.001))
        {
            /*  publish control command   */
            mavros_interface.pub_att_thrust_cmd(U.q_d, U.U1);
            last_ctrol_timestamp = ros::Time::now();
        }

        if (use_logger && ctrl_logger.is_open())
        {
            ctrl_logger << ros::Time::now().toNSec() << ',';
            ctrl_logger << ctrl_res.res(0) << ',';
            ctrl_logger << ctrl_res.res(1) << ',';
            ctrl_logger << ctrl_res.res(2) << ',';
            ctrl_logger << U.U1 << ',';
            ctrl_logger << U.q_d.w() << ',';
            ctrl_logger << U.q_d.x() << ',';
            ctrl_logger << U.q_d.y() << ',';
            ctrl_logger << U.q_d.z() << std::endl;
        }
    }
    else
    {
        ctrl_core.reset();
        mavros_interface.pub_att_thrust_cmd(Eigen::Quaterniond::Identity(), 0.0f);
    }
}

Controller::U_s Controller::cal_Rd_thrust(const PID_ctrl<cmd_s, State_s>::res_s &in_ctrl_res, const cmd_s &_ref)
{
    PID_ctrl<cmd_s, State_s>::res_s ctrl_res = in_ctrl_res;
    U_s res;
    res.header = ctrl_res.header;
    float _yaw_d = _ref.yaw_d;
    if (ros::Time::now() - ctrl_res.header < ros::Duration(0.5f))
    {
        double tilt_max = 30.0f / 180.0f * M_PI;
        double thrust_xy_len = sqrtf(ctrl_res.res(0) * ctrl_res.res(0) + ctrl_res.res(1) * ctrl_res.res(1));
        if (thrust_xy_len > 0.01f)
        {
            double thrust_xy_max = -ctrl_res.res(2) * tanf(tilt_max);
            if (thrust_xy_len > thrust_xy_max)
            {
                double k = thrust_xy_max / thrust_xy_len;
                ctrl_res.res(0) *= k;
                ctrl_res.res(1) *= k;
            }
        }

        /* get R */
        Eigen::Matrix3d _R;
        get_dcm_from_q(_R, get_state().att_q);

        /* get U1 */
        double real_U1 = -ctrl_res.res.transpose() * _R.col(2);
        res.U1 = real_U1 / ONE_G * ctrl_core.get_hover_thrust();

        /* get body_z */
        Eigen::Vector3d _body_z;
        if (ctrl_res.res.norm() > 0.001f)
        {
            _body_z = -ctrl_res.res.normalized();
        }
        else
        {
            _body_z << 0.0f, 0.0f, 1.0f;
        }

        /* get y_C */
        Eigen::Vector3d _y_C(-sin(_yaw_d), cos(_yaw_d), 0.0f);

        /* get body_x */
        Eigen::Vector3d _body_x;
        if (fabsf(_body_z(2)) > 0.0001f)
        {
            _body_x = _y_C.cross(_body_z);
            if (_body_z(2) < 0)
            {
                _body_x = -_body_x;
            }
            _body_x = _body_x.normalized();
        }
        else
        {
            _body_x << 1.0f, 0.0f, 0.0f;
        }

        /* get body_y */
        Eigen::Vector3d _body_y = _body_z.cross(_body_x);
        _body_y = _body_y.normalized();

        /* get R_d */
        Eigen::Matrix3d _R_d;
        _R_d.col(0) = _body_x;
        _R_d.col(1) = _body_y;
        _R_d.col(2) = _body_z;

        /* get q_d */
        get_q_from_dcm(res.q_d, _R_d);
    }
    else
    {
        res.reset();
    }
    return res;
}

void Controller::arm_disarm_vehicle(const bool &arm)
{
    if (arm)
    {
        ROS_INFO("vehicle will be armed!");
        if(use_logger)
            start_logger(ros::Time::now(), uav_id);
        if (mavros_interface.set_arm_and_offboard())
        {
            ROS_INFO("done!");
        }
    }
    else
    {
        ROS_INFO("vehicle will be disarmed!");
        if (mavros_interface.set_disarm())
        {
            ROS_INFO("done!");
        }
        if (use_logger && ctrl_logger.is_open())
        {
            ctrl_logger.close();
        }
    }
}

void Controller::set_hover_pos(const Eigen::Vector3d &pos, const float &yaw)
{
    pthread_mutex_lock(&ctrl_mutex);
    status_ref.header = ros::Time::now();
    status_ref.valid = true;
    status_ref.pos_d = pos;
    status_ref.yaw_d = yaw;
    status_ref.cmd_mask = P_C_V;
    pthread_mutex_unlock(&ctrl_mutex);
}

bool Controller::arm_disarm_srv_handle(ctrl_msg::SetArm::Request &req,
                                       ctrl_msg::SetArm::Response &res)
{
    bool arm_req = req.armed;
    arm_disarm_vehicle(arm_req);
    res.res = true;
    return true;
}

bool Controller::hover_pos_srv_handle(ctrl_msg::SetHover::Request &req,
                                      ctrl_msg::SetHover::Response &res)
{
    Eigen::Vector3d pos_d;
    pos_d << req.x_ned, req.y_ned, req.z_ned;
    float yaw_d = req.yaw;
    set_hover_pos(pos_d, yaw_d);
    res.res = true;
    return true;
}
bool Controller::takeoff_land_srv_handle(ctrl_msg::SetTakeoffLand::Request &req,
                                         ctrl_msg::SetTakeoffLand::Response &res)
{
    Eigen::Vector3d pos_d;
    State_s state_now = get_state();
    Eigen::Vector3d euler;
    get_euler_from_q(euler, state_now.att_q);
    /*  the desired yaw in the process of takeoff   */
    float yaw_d = euler(2);
    if (req.takeoff)
    {
        pos_d << state_now.Pos(0), state_now.Pos(1), state_now.Pos(2) + 0.05f;
        set_hover_pos(pos_d, yaw_d);

        std::cout << "takeoff process start" << std::endl;
        arm_disarm_vehicle(true); // Arm uav
        float Pos_d_z = state_now.Pos(2);
        float takeoff_vel = 0.8f;
        float takeoff_ddz = takeoff_vel / 20.0f;
        ros::Rate takeoff_loop(20);
        std::cout << "takeoff altitude: " << req.takeoff_altitude << " m" << std::endl;
        std::cout << "takeoff velocity: " << takeoff_vel << " m/s" << std::endl;
        ros::Time start_takeoff_task_time = ros::Time::now();
        while (ros::ok() && ros::Time::now() - start_takeoff_task_time < ros::Duration(8.0))
        {
            pos_d(2) = Pos_d_z;
            set_hover_pos(pos_d, yaw_d);
            Pos_d_z -= takeoff_ddz;
            if (Pos_d_z < -req.takeoff_altitude)
            {
                std::cout << "takeoff process done" << std::endl;
                traj_ctrl_valid = true;
                break;
            }
            takeoff_loop.sleep();
        }
    }
    else
    {
        traj_ctrl_valid = false;
        pos_d << state_now.Pos(0), state_now.Pos(1), state_now.Pos(2); // + 0.5f;
        ros::Time start_land_task_time = ros::Time::now();
        ros::Rate land_loop(20.0);
        float temp_Pos_d_z = state_now.Pos(2);
        bool land_flag = false;
        while (ros::ok() && ros::Time::now() - start_land_task_time < ros::Duration(8.0))
        {
            temp_Pos_d_z += 0.5f / 20.0f;
            pos_d(2) = temp_Pos_d_z;
            set_hover_pos(pos_d, yaw_d);
            if(use_distance_sensor)
            {
                pthread_mutex_lock(&lidar_data_mutex);
                float temp_lidar_z = downward_lidar_data.range;
                pthread_mutex_unlock(&lidar_data_mutex);
                land_flag = (temp_lidar_z < 0.11f && temp_lidar_z != 0.0f);
                std::cout<<"temp_lidar_z "<<temp_lidar_z<<std::endl;
            }
            else
            {
                state_now = get_state();
                land_flag = (fabs(state_now.Pos(2)) < 0.1f && fabs(state_now.Vel(2)) < 1.0f);
            }
            if (land_flag)
            {
                ROS_INFO("detect land: disarm");
                arm_disarm_vehicle(false); // Arm uav
                break;
            }
            land_loop.sleep();
        }
    }
    res.res = true;
    return true;
}

void Controller::ctrl_ref_cb(const ctrl_msg::ctrl_ref &msg)
{
    if (traj_ctrl_valid)
    {
        pthread_mutex_lock(&ctrl_mutex);
        status_ref.valid = true;
        status_ref.header = msg.header.stamp;
        status_ref.pos_d << msg.pos_ref[0], msg.pos_ref[1], msg.pos_ref[2];
        status_ref.vel_d << msg.vel_ref[0], msg.vel_ref[1], msg.vel_ref[2];
        status_ref.acc_d << msg.acc_ref[0], msg.acc_ref[1], msg.acc_ref[2];
        status_ref.yaw_d = msg.yaw_ref;
        status_ref.cmd_mask = msg.ref_mask;
        pthread_mutex_unlock(&ctrl_mutex);
    }
}

void Controller::down_ward_lidar_cb(const sensor_msgs::RangePtr msg) {
    pthread_mutex_lock(&lidar_data_mutex);
    downward_lidar_data = *msg;
    pthread_mutex_unlock(&lidar_data_mutex);
}

void Controller::start_logger(const ros::Time &t, const int &id)
{
    if(use_logger)
    {
        std::string temp_file_name = logger_file_name + "UAV_";
        temp_file_name += std::to_string(id);
        temp_file_name += "/ctrl_logger";
        temp_file_name += getTime_string();
        temp_file_name += ".csv";
        std::cout << "controller logger: " << temp_file_name << std::endl;
        if (ctrl_logger.is_open())
        {
            ctrl_logger.close();
        }
        ctrl_logger.open(temp_file_name.c_str(), std::ios::out);
        if (!ctrl_logger.is_open())
        {
            std::cout << "cannot open the logger." << std::endl;
        }
        else
        {
            ctrl_logger << "timestamp" << ',';
            ctrl_logger << "ctrl_output_x" << ',';
            ctrl_logger << "ctrl_output_y" << ',';
            ctrl_logger << "ctrl_output_z" << ',';
            ctrl_logger << "U1" << ',';
            ctrl_logger << "q_d_w" << ',';
            ctrl_logger << "q_d_x" << ',';
            ctrl_logger << "q_d_y" << ',';
            ctrl_logger << "q_d_z" << std::endl;
        }
    }
}

std::string Controller::getTime_string()
{
    time_t timep;
    timep = time(0);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
    return tmp;
}
