//
// Created by tc on 24-12-20.
//
#ifndef BASIC_CONTROL_HPP
#define BASIC_CONTROL_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "airsim_ros/VelCmd.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_msgs/GoalSet.h"
#include "airsim_ros/RotorPWM.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav_msgs/Odometry.h>

#endif //BASIC_CONTROL_HPP

class BasicControl {
private:
    airsim_ros::RotorPWM pwm_cmd;

    ros::Subscriber rc_channel1_suber;
    ros::Subscriber rc_channel2_suber;
    ros::Subscriber rc_channel3_suber;
    ros::Subscriber rc_channel4_suber;
    ros::Subscriber rc_channel5_suber;
    ros::Subscriber rc_channel6_suber;

    ros::Subscriber clicked_point_suber;

    ros::Subscriber imu_suber;
	ros::Subscriber pose_suber;

    ros::Publisher pwm_publisher;

    ros::Publisher rate_x_target_publisher;
    ros::Publisher rate_y_target_publisher;
    ros::Publisher rate_z_target_publisher;
    ros::Publisher rate_x_real_publisher;
    ros::Publisher rate_y_real_publisher;
    ros::Publisher rate_z_real_publisher;
    ros::Publisher quad_goal_publisher;

    ros::Timer rc_mode_timer;
    ros::Timer pwm_send_timer;

    void channel1_callback(const std_msgs::Float32::ConstPtr& msg);
    void channel2_callback(const std_msgs::Float32::ConstPtr& msg);
    void channel3_callback(const std_msgs::Float32::ConstPtr& msg);
    void channel4_callback(const std_msgs::Float32::ConstPtr& msg);
    void channel5_callback(const std_msgs::Float32::ConstPtr& msg);
    void channel6_callback(const std_msgs::Float32::ConstPtr& msg);
	void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void rc_mode_check_callback(const ros::TimerEvent& event);
	void pwm_send_callback(const ros::TimerEvent& event);

    void rviz_clicked_point_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

public:
     BasicControl(ros::NodeHandle *nh);
     ~BasicControl();
};
