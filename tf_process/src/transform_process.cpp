//
// Created by bismarck on 12/11/22.
//
#define USE_CUSTOM_LASER2SCAN

#include <cmath>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Time imu_stamp;
tf2_ros::Buffer tfBuffer;

ros::Publisher lio_pub;

void project2plane_callback(const ros::TimerEvent&){    //将3D位置投影到2D地图上用于导航
    static tf2_ros::TransformBroadcaster br;
    tf2::Quaternion q;
    geometry_msgs::TransformStamped trans;
//    q.setRPY(0, 0, 0);
    q.setRPY(0, 3.1416, 3.1416);

    trans.header.frame_id = "odom";
    trans.child_frame_id = "camera_init";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
    trans.transform.translation.x = 0;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    br.sendTransform(trans);

//    q.setRPY(0, 3.1416, 3.1416);
    trans.header.frame_id = "body";
    trans.child_frame_id = "base_link";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
    trans.transform.translation.x = 0;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    br.sendTransform(trans);

    trans.header.frame_id = "map";
    trans.child_frame_id = "odom_map";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = 0;
    trans.transform.rotation.y = 0;
    trans.transform.rotation.z = 0;
    trans.transform.rotation.w = 1;
    trans.transform.translation.x = 0;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    br.sendTransform(trans);



    geometry_msgs::TransformStamped base2map;
    try {
        base2map = tfBuffer.lookupTransform("odom", "body", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("LIO Get TF ERROR!");
        return;
    }
    nav_msgs::Odometry odom_msg;
    // Fill in the header
    odom_msg.header.stamp = base2map.header.stamp;
    odom_msg.header.frame_id = "odom";  // e.g., "world"s
    odom_msg.child_frame_id = "base_link";  // e.g., "base_link"

    // Fill in the pose from the TransformStamped message
    odom_msg.pose.pose.position.x = base2map.transform.translation.x;
    odom_msg.pose.pose.position.y = base2map.transform.translation.y;
    odom_msg.pose.pose.position.z = base2map.transform.translation.z;

    odom_msg.pose.pose.orientation = base2map.transform.rotation;

    // Optionally, set velocity to zero if unavailable (TransformStamped doesn't contain velocity)
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Publish the Odometry message
    lio_pub.publish(odom_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.02), project2plane_callback);
    lio_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/lio", 10);
    ros::spin();
    return 0;
}
