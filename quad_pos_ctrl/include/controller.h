#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define USE_LOGGER 1

#include "state_estimate_vio.h"
#include "mavros_interface.h"
// #include <boost/thread.hpp>
#include "PID_ctrl.h"
#include "geometry_math_type.h"

// srv
#include "ctrl_msg/SetArm.h"
#include "ctrl_msg/SetHover.h"
#include "ctrl_msg/SetTakeoffLand.h"
// msg
#include "ctrl_msg/ctrl_ref.h"
#include <sensor_msgs/Range.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <iostream>
#include <fstream>
#include <time.h>

#define ONE_G 9.78

void *start_controller_loop_thread(void *args);

class Controller : public State_Estimate_Vio
{
public:
    Controller(int rigid_id) : State_Estimate_Vio(rigid_id),
                               mavros_interface(rigid_id),
                               ctrl_core(rigid_id),
                               uav_id(rigid_id),
                               nh_("~controller")
    {
        already_running = false;
        pthread_mutex_init(&ctrl_mutex, NULL);
        arm_status.reset();

        arm_srv = nh_.advertiseService("arm_disarm", &Controller::arm_disarm_srv_handle, this);
        hoverpos_srv = nh_.advertiseService("hover_pos", &Controller::hover_pos_srv_handle, this);
        takeoff_land_srv = nh_.advertiseService("takeoff_land", &Controller::takeoff_land_srv_handle, this);

        ctrl_ref_sub = nh_.subscribe("ctrl_ref", 1, &Controller::ctrl_ref_cb, this, ros::TransportHints().tcpNoDelay());

        nh_.param<bool>("/use_distance_sensor", use_distance_sensor, false);
        if (use_distance_sensor)
        {
            pthread_mutex_init(&lidar_data_mutex, NULL);
            std::string rangefinder_topic_name = "/mavros";
            rangefinder_topic_name += std::to_string(rigid_id);
            rangefinder_topic_name += "/distance_sensor/rangefinder_pub";
            down_ward_lidar_sub = nh_.subscribe(rangefinder_topic_name, 1, &Controller::down_ward_lidar_cb, this, ros::TransportHints().tcpNoDelay());
        }

        nh_.param<bool>("/use_logger", use_logger, true);
        nh_.param<int>("/frequency", frequency, 50);
        nh_.param<std::string>("/controller_logger_file_name", logger_file_name, "/home/nvidia/work/");

        traj_ctrl_valid = false;

        int result = pthread_create(&ctrl_tid, NULL, &start_controller_loop_thread, this);
        if (result)
            throw result;
    }

    ~Controller()
    {
        pthread_join(ctrl_tid, NULL);
    }

    typedef struct U_res
    {
        ros::Time header;
        Eigen::Quaterniond q_d;
        double U1;

        void reset()
        {
            q_d = Eigen::Quaterniond::Identity();
            U1 = 0.0f;
        }

        U_res()
        {
            header = ros::Time::now();
            reset();
        }
    } U_s;

    typedef struct ctrl_cmd
    {
        ros::Time header;
        bool valid;
        Eigen::Vector3d pos_d;
        Eigen::Vector3d vel_d;
        Eigen::Vector3d acc_d;
        float yaw_d;
        uint8_t cmd_mask; /* |1: position ctrl valied |2: velocity ctrl valied |3: acc ctrl valied |..|8: */
        ctrl_cmd()
        {
            header = ros::Time::now();
            pos_d = Eigen::Vector3d::Zero();
            vel_d = Eigen::Vector3d::Zero();
            acc_d = Eigen::Vector3d::Zero();
            yaw_d = 0.0f;
            cmd_mask = 0;
            valid = false;
        }
    } cmd_s;

    typedef struct arm_status_t
    {
        ros::Time header;
        bool armed;
        void reset()
        {
            armed = false;
        }
        arm_status_t()
        {
            reset();
        }
    } arm_s;

    void start_logger(const ros::Time &t, const int &id);
    std::string getTime_string();

    void arm_disarm_vehicle(const bool &arm);
    void set_hover_pos(const Eigen::Vector3d &pos, const float &yaw);

    void start_controller_loop()
    {
        if (already_running)
        {
            fprintf(stderr, "controller loop already running!\n");
        }
        else
        {
            controller_loop();
        }
    }

    bool arm_disarm_srv_handle(ctrl_msg::SetArm::Request &req,
                               ctrl_msg::SetArm::Response &res);

    bool hover_pos_srv_handle(ctrl_msg::SetHover::Request &req,
                              ctrl_msg::SetHover::Response &res);

    bool takeoff_land_srv_handle(ctrl_msg::SetTakeoffLand::Request &req,
                                 ctrl_msg::SetTakeoffLand::Response &res);

    void ctrl_ref_cb(const ctrl_msg::ctrl_ref &msg);

    void down_ward_lidar_cb(const sensor_msgs::RangePtr msg);

private:
    ros::NodeHandle nh_;
    void controller_loop();
    void one_step(const cmd_s &_ref);
    U_s cal_Rd_thrust(const PID_ctrl<cmd_s, State_s>::res_s &ctrl_res, const cmd_s &_ref);
    pthread_t ctrl_tid;
    cmd_s status_ref;
    bool already_running;
    arm_s arm_status;
    sensor_msgs::Range downward_lidar_data;

    int uav_id;
    int frequency;
    bool use_distance_sensor;
    bool use_logger;
    bool traj_ctrl_valid;

    /* service list */
    ros::ServiceServer arm_srv;
    ros::ServiceServer hoverpos_srv;
    ros::ServiceServer takeoff_land_srv;

    ros::Subscriber ctrl_ref_sub;

    ros::Subscriber down_ward_lidar_sub;

    ros::Time last_ctrol_timestamp;

    Mavros_Interface mavros_interface;

    pthread_mutex_t ctrl_mutex;

    pthread_mutex_t lidar_data_mutex;

    // Logger ctrl_logger;
    std::string logger_file_name;
    std::ofstream ctrl_logger;

    /* choose the ctrl method */
    PID_ctrl<cmd_s, State_s> ctrl_core;
};

void *
start_controller_loop_thread(void *args)
{
    Controller *controller = (Controller *)args;
    controller->start_controller_loop();
    return NULL;
}

#endif
