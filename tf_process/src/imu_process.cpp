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

#define DT 0.01  // Time step (assumed to be 0.1s for this example)
#define Q 0.01  // Process noise covariance (adjust as needed)
#define R_POS 1.0  // Measurement noise covariance for position
#define R_ACC 0.1  // Measurement noise covariance for acceleration

// Kalman filter state (position and velocity)
typedef struct {
    float x[2];  // x[0] = position, x[1] = velocity
    float P[2][2];  // Covariance matrix
} KalmanState;

KalmanState kf_x;
KalmanState kf_y;
KalmanState kf_z;

float real_world_v_x, real_world_v_y, real_world_v_z;
float real_world_acc_x, real_world_acc_y, real_world_acc_z;
float real_world_pre_v_x, real_world_pre_v_y, real_world_pre_v_z;
float real_world_pre_x, real_world_pre_y, real_world_pre_z;

int initialized = 0;

void kalman_init(KalmanState *kf, float initial_position, float initial_velocity) {
    // Initialize state
    kf->x[0] = initial_position;
    kf->x[1] = initial_velocity;

    // Initialize covariance matrix (assumes high uncertainty initially)
    kf->P[0][0] = 1.0;  // Position uncertainty
    kf->P[0][1] = 0.0;  // Cross-correlation between position and velocity
    kf->P[1][0] = 0.0;  // Cross-correlation between position and velocity
    kf->P[1][1] = 1.0;  // Velocity uncertainty
}

void kalman_predict(KalmanState *kf, float acceleration) {
    // State prediction: x' = Ax + Bu, where u is acceleration
    kf->x[0] += kf->x[1] * DT + 0.5 * acceleration * DT * DT;  // Predict position
    kf->x[1] += acceleration * DT;  // Predict velocity

    // Covariance prediction: P' = AP + PA^T + Q
    kf->P[0][0] += kf->P[0][1] * DT + DT * kf->P[1][0] + Q;
    kf->P[0][1] += kf->P[0][1] * DT;
    kf->P[1][0] += kf->P[1][0] * DT;
    kf->P[1][1] += Q;
}

void kalman_correct(KalmanState *kf, float measured_position) {
    // Measurement update
    float y = measured_position - kf->x[0];  // Innovation (residual)

    // Calculate Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
    float S = kf->P[0][0] + R_POS;  // Measurement residual covariance
    float K[2];  // Kalman gain
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Update state estimate
    kf->x[0] += K[0] * y;  // Update position
    kf->x[1] += K[1] * y;  // Update velocity

    // Update covariance estimate
    float P00_new = kf->P[0][0] - K[0] * kf->P[0][0];
    float P01_new = kf->P[0][1] - K[0] * kf->P[0][1];
    float P10_new = kf->P[1][0] - K[1] * kf->P[0][0];
    float P11_new = kf->P[1][1] - K[1] * kf->P[1][0];

    kf->P[0][0] = P00_new;
    kf->P[0][1] = P01_new;
    kf->P[1][0] = P10_new;
    kf->P[1][1] = P11_new;
}

ros::Time imu_stamp;
tf2_ros::Buffer tfBuffer;

ros::Publisher imu_now_pub;
ros::Publisher odom_pub;
ros::Publisher lio_pub;
ros::Publisher real_pose_pub;
ros::Publisher pcl_pub;

float init_pose_x, init_pose_y, init_pose_z;

typedef struct {
    float w, x, y, z;
} Quaternion;

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

void InitialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    init_pose_x = msg->pose.position.x;
    init_pose_y = msg->pose.position.y;
    init_pose_z = msg->pose.position.z;
    init_quat.x = msg->pose.orientation.x;
    init_quat.y = msg->pose.orientation.y;
    init_quat.z = msg->pose.orientation.z;
    init_quat.w = msg->pose.orientation.w;
    if(initialized == 0) {
        kalman_init(&kf_x, init_pose_x, 0.0f);
        kalman_init(&kf_y, init_pose_y, 0.0f);
        kalman_init(&kf_z, init_pose_z, 0.0f);
        initialized = 1;
    }
}

void RealPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Create an Odometry message
    nav_msgs::Odometry odom_msg;

    // Fill in the header
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "map";  // Set your desired frame_id, e.g., "odom"
    odom_msg.child_frame_id = "base_link";
    // Fill in the pose

    odom_msg.pose.pose.position.x = msg->pose.position.x;
    odom_msg.pose.pose.position.y = msg->pose.position.y;
    odom_msg.pose.pose.position.z = msg->pose.position.z;
    odom_msg.pose.pose.orientation.x = msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.y = msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.z = msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = msg->pose.orientation.w;

    // Optionally, set twist (linear and angular velocity) to zero if unavailable
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    real_world_v_x = (msg->pose.position.x - real_world_pre_x) * 100.0f;
    real_world_v_y = (msg->pose.position.y - real_world_pre_y) * 100.0f;
    real_world_v_z = (msg->pose.position.z - real_world_pre_z) * 100.0f;
    real_world_pre_x = msg->pose.position.x;
    real_world_pre_y = msg->pose.position.y;
    real_world_pre_z = msg->pose.position.z;
    real_world_acc_x = (real_world_v_x - real_world_pre_v_x) * 100.0f;
    real_world_acc_y = (real_world_v_y - real_world_pre_v_y) * 100.0f;
    real_world_acc_z = (real_world_v_z - real_world_pre_v_z) * 100.0f;
    real_world_pre_v_x = real_world_v_x;
    real_world_pre_v_y = real_world_v_y;
    real_world_pre_v_z = real_world_v_z;


    // Publish the Odometry message
    real_pose_pub.publish(odom_msg);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // Create an Odometry message
    nav_msgs::Odometry odom_msg;

    // Fill in the header
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "map";  // Set your desired frame_id, e.g., "odom"
    odom_msg.child_frame_id = "base_link";
    // Fill in the pose
    odom_msg.pose.pose.position.x = msg->pose.position.x;
    odom_msg.pose.pose.position.y = msg->pose.position.y;
    odom_msg.pose.pose.position.z = msg->pose.position.z;
    odom_msg.pose.pose.orientation.x = msg->pose.orientation.x;
    odom_msg.pose.pose.orientation.y = msg->pose.orientation.y;
    odom_msg.pose.pose.orientation.z = msg->pose.orientation.z;
    odom_msg.pose.pose.orientation.w = msg->pose.orientation.w;

    // Optionally, set twist (linear and angular velocity) to zero if unavailable
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // Publish the Odometry message
    odom_pub.publish(odom_msg);
    if(initialized != 0) {
        kalman_correct(&kf_x, msg->pose.position.x);
        kalman_correct(&kf_y, msg->pose.position.y);
        kalman_correct(&kf_z, msg->pose.position.z);
    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  imu_stamp = msg->header.stamp;
    sensor_msgs::Imu new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();  // Update the timestamp to the current time
    new_msg.header.frame_id = "base_link";
    imu_now_pub.publish(new_msg);
    Quaternion q;
    q.x = msg->orientation.x;
    q.y = msg->orientation.y;
    q.z = msg->orientation.z;
    q.w = msg->orientation.w;
    Quaternion world_quat = multiply_quaternion(&init_quat, &q);
    tf2::Quaternion world_to_body_quat(world_quat.x, world_quat.y, world_quat.z, world_quat.w);
    tf2::Quaternion body_to_world_quat = world_to_body_quat.inverse();
    tf2::Vector3 acc_body(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
//    ROS_INFO("world_quat: %f %f %f %f", world_quat.x, world_quat.y, world_quat.z, world_quat.w);
    tf2::Vector3 acc_world = tf2::quatRotate(body_to_world_quat, acc_body);

    if(initialized != 0) {
        kalman_predict(&kf_x, acc_world.x());
        kalman_predict(&kf_y, acc_world.y());
        kalman_predict(&kf_z, acc_world.z());
        ROS_INFO("imu acc: x=%f, y=%f, z=%f", acc_world.x(), acc_world.y(), acc_world.z());
//        ROS_INFO("speed: x=%f, y=%f, z=%f", kf_x.x[1], kf_y.x[1], kf_z.x[1]);
//        std::cout << "x position:" << kf_x.x[0] << "x velocity:" << kf_x.x[1] << std::endl;
//        std::cout << "y position:" << kf_y.x[0] << "y velocity:" << kf_y.x[1] << std::endl;
//        std::cout << "z position:" << kf_z.x[0] << "z velocity:" << kf_z.x[1] << std::endl;
    }
}

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    sensor_msgs::PointCloud2 new_msg = *msg;  // Copy the original message

    // Update the timestamp to the current time
    new_msg.header.stamp = ros::Time::now();
    new_msg.header.frame_id = "base_link";
    // Republish the message with the new timestamp
    pcl_pub.publish(new_msg);
}

void project2plane_callback(const ros::TimerEvent&){    //将3D位置投影到2D地图上用于导航
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans;

    trans.header.frame_id = "map";
    trans.child_frame_id = "odom";
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
    ros::Subscriber pcl_sub = pnh.subscribe("/airsim_node/drone_1/lidar", 10, pclCallback);
    ros::Subscriber init_sub = pnh.subscribe("/airsim_node/initial_pose", 10, InitialPoseCallback);

    imu_now_pub = pnh.advertise<sensor_msgs::Imu>("/ekf/imu_now", 10);
    pcl_pub = pnh.advertise<sensor_msgs::PointCloud2>("/ekf/pcl", 10);
    odom_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/uwb", 10);
    real_pose_pub = pnh.advertise<geometry_msgs::PoseStamped>("/debug/real_pose_odom", 10);
    lio_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/lio", 10);

    ros::spin();
    return 0;
}
