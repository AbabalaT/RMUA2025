#define USE_CUSTOM_LASER2SCAN
#include <list>
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
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Float32MultiArray.h>

ros::Time imu_stamp;
tf2_ros::Buffer tfBuffer;

ros::Publisher lio_pub;
ros::Publisher pcl_pose_pub;
ros::Publisher cmd_pub;

quadrotor_msgs::PositionCommand rcv_cmd_msg;

typedef struct
{
    float w, x, y, z;
} Quaternion;

float mahony_quaternion[4];

Quaternion multiply_quaternion(Quaternion* q1, Quaternion* q2)
{
    Quaternion result;
    if (q1->w < 0.0f)
    {
        q1->w = -1.0f * q1->w;
        q1->x = -1.0f * q1->x;
        q1->y = -1.0f * q1->y;
        q1->z = -1.0f * q1->z;
    }
    if (q2->w < 0.0f)
    {
        q2->w = -1.0f * q2->w;
        q2->x = -1.0f * q2->x;
        q2->y = -1.0f * q2->y;
        q2->z = -1.0f * q2->z;
    }
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    if (result.w < 0.0f)
    {
        result.w = -result.w;
        result.x = -result.x;
        result.y = -result.y;
        result.z = -result.z;
    }
    return result;
}

Quaternion quaternion_conjugate(Quaternion q)
{
    Quaternion result = {q.w, -q.x, -q.y, -q.z};
    return result;
}

// Function to rotate a vector by a quaternion
void rotateVectorByQuaternion(Quaternion q, float v[3], float result[3])
{
    // Convert the vector into a quaternion with w = 0
    Quaternion q_vec = {0.0f, v[0], v[1], v[2]};

    // Calculate the conjugate of the quaternion
    Quaternion q_conj = quaternion_conjugate(q);

    // Perform the rotation: q * v * q^-1
    Quaternion temp = multiply_quaternion(&q, &q_vec); // q * v
    Quaternion q_result = multiply_quaternion(&temp, &q_conj); // (q * v) * q^-1

    // The result quaternion's x, y, z are the rotated vector components
    result[0] = q_result.x;
    result[1] = q_result.y;
    result[2] = q_result.z;
}

Quaternion init_quat;
Quaternion world_quat;
Quaternion world_quat_no_rotation;

Quaternion body_axis_z(Quaternion measure, float theta)
{
    Quaternion rotation;
    rotation.w = cos(theta / 2.0f);
    rotation.x = sin(theta / 2.0f);
    rotation.y = 0.0f;
    rotation.z = 0.0f;

    Quaternion result = multiply_quaternion(&measure, &rotation);
    return result;
}

float odom_pos[3];

struct drone_cmd
{
    float pos[3];
    float vel[3];
    float acc[3];
    float yaw;
    float w_yaw;
};

float point_distance(const float p1[], const float p2[])
{
    float dist = 0;
    for (int i = 0; i < 3; i++)
    {
        dist += (p2[i] - p1[i]) * (p2[i] - p1[i]);
    }
    return sqrt(dist);
}

std::list<drone_cmd> drone_cmds;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pos[0] = msg->pose.pose.position.x;
    odom_pos[1] = msg->pose.pose.position.y;
    odom_pos[2] = msg->pose.pose.position.z;
    nav_msgs::Odometry new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();
    new_msg.header.frame_id = "world";
    // new_msg.child_frame_id = "base_link";
    new_msg.pose.pose.position.x = new_msg.pose.pose.position.x - 750.0;
    lio_pub.publish(new_msg);
    new_msg.child_frame_id = "body";

    // new_msg.pose.pose.orientation.x = msg->pose.pose.orientation.y;
    // new_msg.pose.pose.orientation.y = msg->pose.pose.orientation.x;
    // new_msg.pose.pose.orientation.z = -msg->pose.pose.orientation.z;
    // new_msg.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    Quaternion q_measure;

    q_measure.w = msg->pose.pose.orientation.w;
    q_measure.x = msg->pose.pose.orientation.x;
    q_measure.y = msg->pose.pose.orientation.y;
    q_measure.z = msg->pose.pose.orientation.z;

    Quaternion q_measured = body_axis_z(q_measure, 3.14159265358);

    new_msg.pose.pose.orientation.x = q_measured.x;
    new_msg.pose.pose.orientation.y = q_measured.y;
    new_msg.pose.pose.orientation.z = q_measured.z;
    new_msg.pose.pose.orientation.w = q_measured.w;

    pcl_pose_pub.publish(new_msg);
}

void drone_cmd_callback(const quadrotor_msgs::PositionCommandConstPtr& msg)
{
    auto received_drone_cmd = std::make_shared<drone_cmd>();
    received_drone_cmd->pos[0] = msg->position.x;
    received_drone_cmd->pos[1] = msg->position.y;
    received_drone_cmd->pos[2] = msg->position.z;
    received_drone_cmd->vel[0] = msg->velocity.x;
    received_drone_cmd->vel[1] = msg->velocity.y;
    received_drone_cmd->vel[2] = msg->velocity.z;
    received_drone_cmd->acc[0] = msg->acceleration.x;
    received_drone_cmd->acc[1] = msg->acceleration.y;
    received_drone_cmd->acc[2] = msg->acceleration.z;
    received_drone_cmd->yaw = msg->yaw;
    received_drone_cmd->w_yaw = msg->yaw_dot;
    // drone_cmds.push_back(received_drone_cmd);
    float cmd_array[11] = {
        (received_drone_cmd->pos[0]) + 750, received_drone_cmd->pos[1], received_drone_cmd->pos[2],
        received_drone_cmd->vel[0], received_drone_cmd->vel[1], received_drone_cmd->vel[2],
        received_drone_cmd->acc[0], received_drone_cmd->acc[1], received_drone_cmd->acc[2],
        received_drone_cmd->yaw, received_drone_cmd->w_yaw
    };
    std_msgs::Float32MultiArray cmd_msg;
    cmd_msg.data.assign(cmd_array, cmd_array + 11);
    cmd_pub.publish(cmd_msg);
}

void pub_required_cmd(const ros::TimerEvent&)
{
    static bool approaching = false;
    if (drone_cmds.size() > 1)
    {
        auto cmd_selected = drone_cmds.begin();
        auto cmd_iter = std::next(cmd_selected);
        auto cmd_distance = point_distance(cmd_selected->pos, odom_pos);
        for (int i = 0; i < 150; i++)
        {
            auto iter_distance = point_distance(cmd_iter->pos, odom_pos);
            if (iter_distance < cmd_distance)
            {
                cmd_selected = cmd_iter;
                cmd_distance = iter_distance;
            }
            if (cmd_iter == drone_cmds.end())
            {
                break;
            }
            cmd_iter++;
        }
        std::cout << "start pop" << std::endl;
        while (1)
        {
            if (cmd_selected == drone_cmds.begin())
            {
                break;
            }
            else
            {
                drone_cmds.pop_front();
            }
        }
        std::cout << "end pop" << std::endl;
        float cmd_array[11] = {
            cmd_selected->pos[0], cmd_selected->pos[1], cmd_selected->pos[2],
            cmd_selected->vel[0], cmd_selected->vel[1], cmd_selected->vel[2],
            cmd_selected->acc[0], cmd_selected->acc[1], cmd_selected->acc[2],
            cmd_selected->yaw, cmd_selected->w_yaw
        };
        if (cmd_distance > 0.6f)
        {
            approaching = true;
        }
        if (cmd_distance < 0.2f)
        {
            approaching = false;
        }
        if (approaching)
        {
            for (int i = 3; i < 9; i++)
            {
                cmd_array[i] = NAN;
            }
            std::cout << "distance to first command is: " << cmd_distance << ", approaching first!" << std::endl;
        }
        std_msgs::Float32MultiArray cmd_msg;
        cmd_msg.data.assign(cmd_array, cmd_array + 11);
        cmd_pub.publish(cmd_msg);
    }
    else
    {
        // std::cout <<"cmd num is"<<drone_cmds.size()<<", so not publish cmd" << std::endl;
    }
}

void project2plane_callback(const ros::TimerEvent&)
{
    //将3D位置投影到2D地图上用于导航
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
    trans.transform.translation.x = -750;
    trans.transform.translation.y = 0;
    trans.transform.translation.z = 0;
    br.sendTransform(trans);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.01), project2plane_callback);
    // ros::Timer timer2 = pnh.createTimer(ros::Duration(0.01), pub_required_cmd);
    lio_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/lio", 10);
    pcl_pose_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/pcl_pose", 10);
    cmd_pub = pnh.advertise<std_msgs::Float32MultiArray>("/exe/cmd", 10);
    ros::Subscriber rc6_sub = pnh.subscribe("/drone_0_planning/pos_cmd", 10, drone_cmd_callback);
    ros::Subscriber pose_sub = pnh.subscribe("/odometry/filtered", 10, poseCallback);

    drone_cmd received_drone_cmd;
    received_drone_cmd.pos[0] = 9999.0;
    received_drone_cmd.pos[1] = 9999.0;
    received_drone_cmd.pos[2] = 9999.0;
    received_drone_cmd.vel[0] = 0;
    received_drone_cmd.vel[1] = 0;
    received_drone_cmd.vel[2] = 0;
    received_drone_cmd.acc[0] = 0;
    received_drone_cmd.acc[1] = 0;
    received_drone_cmd.acc[2] = 0;
    // drone_cmds.push_back(received_drone_cmd);

    ros::spin();
    return 0;
}
