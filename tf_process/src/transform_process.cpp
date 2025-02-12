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
    q.setRPY(0, 3.1416, 3.1416);

    trans.header.frame_id = "base_link";
    trans.child_frame_id = "body";
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
    trans.child_frame_id = "map_odom";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = 0.0;
    trans.transform.rotation.y = 0.0;
    trans.transform.rotation.z = 0.0;
    trans.transform.rotation.w = 1.0;
    trans.transform.translation.x = 0;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    br.sendTransform(trans);

    trans.header.frame_id = "world";
    trans.child_frame_id = "map";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = 0.0;
    trans.transform.rotation.y = 0.0;
    trans.transform.rotation.z = 0.0;
    trans.transform.rotation.w = 1.0;
    trans.transform.translation.x = 0;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    br.sendTransform(trans);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tf_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.01), project2plane_callback);
    lio_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/lio", 10);
    ros::spin();
    return 0;
}
