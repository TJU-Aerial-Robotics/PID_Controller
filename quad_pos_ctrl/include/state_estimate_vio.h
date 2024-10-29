#ifndef STATE_ESTIMATE_VIO_H_
#define STATE_ESTIMATE_VIO_H_

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


class State_Estimate_Vio{
    public:

        typedef struct state_struction {
            ros::Time header;
            Eigen::Vector3d Pos;
            Eigen::Vector3d Vel;
            Eigen::Vector3d Acc;
            Eigen::Quaterniond att_q;
        }State_s;

        void IMU_Odometry_lidar_data_cb(const nav_msgs::Odometry::ConstPtr &msg){
            pthread_mutex_lock(&state_mutex);
            state_.header = msg->header.stamp;
            state_.Pos << msg->pose.pose.position.x, -msg->pose.pose.position.y, -msg->pose.pose.position.z; 
            state_.Vel << msg->twist.twist.linear.x, -msg->twist.twist.linear.y, -msg->twist.twist.linear.z;
            state_.att_q.w() = msg->pose.pose.orientation.w; 
            state_.att_q.x() = msg->pose.pose.orientation.x; 
            state_.att_q.y() = -msg->pose.pose.orientation.y; 
            state_.att_q.z() = -msg->pose.pose.orientation.z; 
            state_.Acc << 0.0, 0.0, 0.0;
            pthread_mutex_unlock(&state_mutex);
        }

        void Local_position_data_cb(const nav_msgs::Odometry::ConstPtr &msg){
            pthread_mutex_lock(&state_mutex);
            state_.header = msg->header.stamp;
            state_.Pos << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z; 
            state_.Vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
            state_.att_q.w() = msg->pose.pose.orientation.w; 
            state_.att_q.x() = msg->pose.pose.orientation.x; 
            state_.att_q.y() = msg->pose.pose.orientation.y; 
            state_.att_q.z() = msg->pose.pose.orientation.z; 
            state_.Acc << 0.0, 0.0, 0.0;
            pthread_mutex_unlock(&state_mutex);
        }

        State_Estimate_Vio(int id) : 
        nh_("~state_estimate"),
        rigidbody_id_(id){
            
            /*  for lidar odometry    */
            std::string IMU_Odometry_lidar_topic_name = "/uav";
            IMU_Odometry_lidar_topic_name += std::to_string(id);
            IMU_Odometry_lidar_topic_name += "/IMU_Odometry_lidar";
            Lidar_odom_sub_ = nh_.subscribe(IMU_Odometry_lidar_topic_name,1,&State_Estimate_Vio::IMU_Odometry_lidar_data_cb,this,ros::TransportHints().tcpNoDelay());

            /*  for local_position in mavros or Gazebo simulation   */
            std::string Mavros_odom_topic_name = "/mavros";
            Mavros_odom_topic_name += std::to_string(id);
            Mavros_odom_topic_name += "/local_position/odom";
            Mavros_odom_sub_ = nh_.subscribe(Mavros_odom_topic_name,1,&State_Estimate_Vio::Local_position_data_cb,this,ros::TransportHints().tcpNoDelay());

            pthread_mutex_init(&state_mutex, NULL);
        }

        ~State_Estimate_Vio() {
            pthread_mutex_destroy(&state_mutex);
        }

        State_s get_state() {
            State_s temp_state_;
            pthread_mutex_lock(&state_mutex);
            temp_state_ = state_;
            pthread_mutex_unlock(&state_mutex);
            return temp_state_; 
        }

    private:
        ros::NodeHandle nh_;
        int rigidbody_id_;
        State_s state_;
        ros::Subscriber Lidar_odom_sub_;
        ros::Subscriber Mavros_odom_sub_;
        pthread_mutex_t state_mutex;
};

#endif
