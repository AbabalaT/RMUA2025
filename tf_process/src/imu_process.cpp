//
// Created by bismarck on 12/11/22.
//
#define USE_CUSTOM_LASER2SCAN

#include <cmath>
#include <iostream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "MahonyAHRS.hpp"

tf2::Quaternion ned2enu_quat;

float real_world_v_x, real_world_v_y, real_world_v_z;
float real_world_acc_x, real_world_acc_y, real_world_acc_z;
float real_world_pre_v_x, real_world_pre_v_y, real_world_pre_v_z;
float real_world_pre_x, real_world_pre_y, real_world_pre_z;

int initialized = 0;

int start_seq = 40;

ros::Time imu_stamp;
tf2_ros::Buffer tfBuffer;

ros::Publisher imu_now_pub;
ros::Publisher odom_pub;
ros::Publisher lio_pub;
ros::Publisher real_pose_pub;
ros::Publisher pcl_pub;
ros::Publisher ahrs_pub;
ros::Publisher uwb_map_pub;

float init_pose_x, init_pose_y, init_pose_z;

typedef struct {
    float w, x, y, z;
} Quaternion;

float mahony_quaternion[4];

Quaternion multiply_quaternion(Quaternion *q1, Quaternion *q2) {
    Quaternion result;
    if(q1->w < 0.0f){
        q1->w = -1.0f * q1->w;
        q1->x = -1.0f * q1->x;
        q1->y = -1.0f * q1->y;
        q1->z = -1.0f * q1->z;
    }
    if(q2->w < 0.0f){
        q2->w = -1.0f * q2->w;
        q2->x = -1.0f * q2->x;
        q2->y = -1.0f * q2->y;
        q2->z = -1.0f * q2->z;
    }
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    if(result.w < 0.0f){
        result.w = -result.w;
        result.x = -result.x;
        result.y = -result.y;
        result.z = -result.z;
    }
    return result;
}

Quaternion quaternion_conjugate(Quaternion q) {
    Quaternion result = {q.w, -q.x, -q.y, -q.z};
    return result;
}

// Function to rotate a vector by a quaternion
void rotateVectorByQuaternion(Quaternion q, float v[3], float result[3]) {
    // Convert the vector into a quaternion with w = 0
    Quaternion q_vec = {0.0f, v[0], v[1], v[2]};

    // Calculate the conjugate of the quaternion
    Quaternion q_conj = quaternion_conjugate(q);

    // Perform the rotation: q * v * q^-1
    Quaternion temp = multiply_quaternion(&q, &q_vec);         // q * v
    Quaternion q_result = multiply_quaternion(&temp, &q_conj); // (q * v) * q^-1

    // The result quaternion's x, y, z are the rotated vector components
    result[0] = q_result.x;
    result[1] = q_result.y;
    result[2] = q_result.z;
}

Quaternion init_quat;
Quaternion world_quat;
Quaternion world_quat_no_rotation;

Quaternion body_axis_z(Quaternion measure, float theta) {
  Quaternion rotation;
  rotation.w = cos(theta/2.0f);
  rotation.x = 0.0f;
  rotation.y = 0.0f;
  rotation.z = sin(theta/2.0f);

  Quaternion result = multiply_quaternion(&measure, &rotation);
  return result;
}

void InitialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    init_pose_x =  msg->pose.position.y;
    init_pose_y =  msg->pose.position.x;
    init_pose_z = -msg->pose.position.z - 0.3f;

    init_quat.x = msg->pose.orientation.y;
    init_quat.y = msg->pose.orientation.x;
    init_quat.z = -msg->pose.orientation.z;
    init_quat.w = msg->pose.orientation.w;
	world_quat_no_rotation = init_quat;
	Quaternion quat = body_axis_z(init_quat, 3.14159265359f / 2.0f);
    init_quat = quat;
}

void RealPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    nav_msgs::Odometry odom_msg;


    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";


    odom_msg.pose.pose.position.x = msg->pose.position.y;
    odom_msg.pose.pose.position.y = msg->pose.position.x;
    odom_msg.pose.pose.position.z = -msg->pose.position.z;

    odom_msg.pose.pose.orientation.x = msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.y = msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.z = -msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = msg->pose.orientation.w;

    Quaternion q_measure;

    q_measure.w = odom_msg.pose.pose.orientation.w;
    q_measure.x = odom_msg.pose.pose.orientation.x;
    q_measure.y = odom_msg.pose.pose.orientation.y;
    q_measure.z = odom_msg.pose.pose.orientation.z;

    Quaternion q_measured = body_axis_z(q_measure, 3.14159265359 / 2.0f);

    odom_msg.pose.pose.orientation.x = q_measured.x;
    odom_msg.pose.pose.orientation.y = q_measured.y;
    odom_msg.pose.pose.orientation.z = q_measured.z;
    odom_msg.pose.pose.orientation.w = q_measured.w;
    real_pose_pub.publish(odom_msg);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = msg->pose.position.y;
    odom_msg.pose.pose.position.y = msg->pose.position.x;
    odom_msg.pose.pose.position.z = -msg->pose.position.z;
    odom_msg.pose.pose.orientation.x = msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.y = msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.z = -msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = msg->pose.orientation.w;

    Quaternion q_measure;

    q_measure.w = odom_msg.pose.pose.orientation.w;
    q_measure.x = odom_msg.pose.pose.orientation.x;
    q_measure.y = odom_msg.pose.pose.orientation.y;
    q_measure.z = odom_msg.pose.pose.orientation.z;

    Quaternion q_measured = body_axis_z(q_measure, 3.14159265359 / 2.0f);

    odom_msg.pose.pose.orientation.x = world_quat.x;
    odom_msg.pose.pose.orientation.y = world_quat.y;
    odom_msg.pose.pose.orientation.z = world_quat.z;
    odom_msg.pose.pose.orientation.w = world_quat.w;

    double conv_1 = 0.1f;
    double conv_2 = 0.01f;

    for(int i = 0; i < 35; i++) {
        odom_msg.pose.covariance[0] = 0.0;
    }

    odom_msg.pose.covariance[0] = conv_1;
    odom_msg.pose.covariance[7] = conv_1;
    odom_msg.pose.covariance[14] = conv_1;
    odom_msg.pose.covariance[21] = conv_2;
    odom_msg.pose.covariance[28] = conv_2;
    odom_msg.pose.covariance[35] = conv_2;

    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom_msg);
    odom_msg.header.frame_id = "map_odom";
    uwb_map_pub.publish(odom_msg);
}

double zero[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int imu_cnt = 0;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  	imu_stamp = ros::Time::now();
	sensor_msgs::Imu new_msg = *msg;
//	new_msg.header.stamp = ros::Time::now();
	new_msg.header.frame_id = "base_link";

    Quaternion q_measure;
    q_measure.w = new_msg.orientation.w;
    q_measure.x = new_msg.orientation.y;
    q_measure.y = new_msg.orientation.x;
    q_measure.z = - new_msg.orientation.z;

    new_msg.orientation.x = q_measure.x;
    new_msg.orientation.y = q_measure.y;
    new_msg.orientation.z = q_measure.z;
    new_msg.orientation.w = q_measure.w;

    double conv_1 = 0.1;
    double conv_2 = 0.5;
    double conv_3 = 1e-5;
    double conv_4 = 1e-5;
    new_msg.angular_velocity_covariance[0] = conv_1;

    new_msg.angular_velocity_covariance[1] = conv_3;
    new_msg.angular_velocity_covariance[2] = conv_3;
    new_msg.angular_velocity_covariance[3] = conv_3;

    new_msg.angular_velocity_covariance[4] = conv_1;

    new_msg.angular_velocity_covariance[5] = conv_3;
    new_msg.angular_velocity_covariance[6] = conv_3;
    new_msg.angular_velocity_covariance[7] = conv_3;

    new_msg.angular_velocity_covariance[8] = conv_1;

    new_msg.linear_acceleration_covariance[0] = conv_2;
    new_msg.linear_acceleration_covariance[4] = conv_2;
    new_msg.linear_acceleration_covariance[8] = conv_2;

    new_msg.linear_acceleration_covariance[1] = conv_4;
    new_msg.linear_acceleration_covariance[2] = conv_4;
    new_msg.linear_acceleration_covariance[3] = conv_4;
    new_msg.linear_acceleration_covariance[5] = conv_4;
    new_msg.linear_acceleration_covariance[6] = conv_4;
    new_msg.linear_acceleration_covariance[7] = conv_4;

//    MahonyAHRSupdateIMU(mahony_quaternion, new_msg.angular_velocity.x, new_msg.angular_velocity.y, new_msg.angular_velocity.z,
//                        new_msg.linear_acceleration.x, new_msg.linear_acceleration.y, new_msg.linear_acceleration.z);
	//ROS_INFO("IMU quaternion: w: %f, x: %f, y:%f, z:%f", mahony_quaternion[0], mahony_quaternion[1], mahony_quaternion[2], mahony_quaternion[3]);

    tf2::Quaternion world_to_body_quat(new_msg.orientation.x, new_msg.orientation.y, new_msg.orientation.z, new_msg.orientation.w);
    tf2::Quaternion body_to_world_quat = world_to_body_quat.inverse();
    tf2::Vector3 acc_body(new_msg.linear_acceleration.x, new_msg.linear_acceleration.y, new_msg.linear_acceleration.z);

    tf2::Vector3 acc_world = tf2::quatRotate(body_to_world_quat, acc_body);
    acc_world.setZ(acc_world.z() + 9.80);
    acc_body = tf2::quatRotate(world_to_body_quat, acc_world);

    new_msg.linear_acceleration.x = acc_body.x();
    new_msg.linear_acceleration.y = -acc_body.y();
    new_msg.linear_acceleration.z = -acc_body.z();


//
//    double temp = new_msg.angular_velocity.x;
    new_msg.angular_velocity.y = -new_msg.angular_velocity.y;
    new_msg.angular_velocity.z = -new_msg.angular_velocity.z;

	if(imu_cnt < 300){
          imu_cnt = imu_cnt+1;
          zero[0] += new_msg.linear_acceleration.x;
          zero[1] += new_msg.linear_acceleration.y;
          zero[2] += new_msg.linear_acceleration.z;
          zero[3] += new_msg.angular_velocity.x;
          zero[4] += new_msg.angular_velocity.y;
          zero[5] += new_msg.angular_velocity.z;
          if(imu_cnt == 300){
			zero[0] = zero[0] / -300.0;
            zero[1] = zero[1] / -300.0;
            zero[2] = zero[2] / -300.0;
            zero[3] = zero[3] / -300.0;
            zero[4] = zero[4] / -300.0;
            zero[5] = zero[5] / -300.0;
            std::cout<<zero[0]<<" "<<zero[1]<<" "<<zero[2]<<" "<<zero[3]<<std::endl;
          }
	}else{
		new_msg.linear_acceleration.x += zero[0];
    	new_msg.linear_acceleration.y += zero[1];
    	new_msg.linear_acceleration.z += zero[2];
		new_msg.angular_velocity.x += zero[3];
    	new_msg.angular_velocity.y += zero[4];
    	new_msg.angular_velocity.z += zero[5];
	}

    imu_now_pub.publish(new_msg);
    Quaternion q_measure_world = multiply_quaternion(&world_quat_no_rotation, &q_measure);
    world_quat = body_axis_z(q_measure_world, 3.14159265359 / 2.0f);
}

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sensor_msgs::PointCloud2 new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();
    new_msg.header.frame_id = "base_link";
    pcl_pub.publish(new_msg);
}

void project2plane_callback(const ros::TimerEvent&){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans;

    trans.header.frame_id = "map";
    trans.child_frame_id = "real_start";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = init_quat.x;
    trans.transform.rotation.y = init_quat.y;
    trans.transform.rotation.z = init_quat.z;
    trans.transform.rotation.w = init_quat.w;
    trans.transform.translation.x = init_pose_x;
    trans.transform.translation.y = init_pose_y;
    trans.transform.translation.z = init_pose_z;
    br.sendTransform(trans);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.02), project2plane_callback);

    ros::Subscriber imu_sub = pnh.subscribe("/airsim_node/drone_1/imu/imu", 10, imuCallback);
    ros::Subscriber pose_sub = pnh.subscribe("/airsim_node/drone_1/gps", 10, poseCallback);
    ros::Subscriber debug_pose = pnh.subscribe("/airsim_node/drone_1/debug/pose_gt", 10, RealPoseCallback);
    //ros::Subscriber pcl_sub = pnh.subscribe("/airsim_node/drone_1/lidar", 10, pclCallback);
    ros::Subscriber init_sub = pnh.subscribe("/airsim_node/initial_pose", 10, InitialPoseCallback);

    imu_now_pub = pnh.advertise<sensor_msgs::Imu>("/ekf/imu_now", 10);
    pcl_pub = pnh.advertise<sensor_msgs::PointCloud2>("/ekf/pcl", 10);
    odom_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/uwb", 10);
    real_pose_pub = pnh.advertise<nav_msgs::Odometry>("/debug/real_pose_odom", 10);
    lio_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/lio", 10);
    ahrs_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/ahrs", 10);
	uwb_map_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/uwb2", 10);

    mahony_quaternion[0] = 1.0;
    mahony_quaternion[1] = 0.0;
    mahony_quaternion[2] = 0.0;
    mahony_quaternion[3] = 0.0;
    ros::spin();
    return 0;
}
