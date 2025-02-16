//
// Created by KevinTC.
//
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_eigen/tf2_eigen.h>

// #include <pcl/features/normal_3d.h>
// #include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/mls.h>

tf2_ros::Buffer tfBuffer;
ros::Publisher pcl_publisher;

inline float float_abs(float x)
{
    if (x > 0)
    {
        return x;
    }
    else
    {
        return -x;
    }
}

typedef struct
{
    float w, x, y, z;
} Quaternion;

float mahony_quaternion[4];

float odom_pos[7];



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

struct PointInt
{
    int p_x, p_y, p_z;
};

float current_x = 0.0, current_y = 0.0, current_z = 0.0;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_pos[0] = msg->pose.pose.position.x - 750.0;
    odom_pos[1] = msg->pose.pose.position.y;
    odom_pos[2] = msg->pose.pose.position.z;
    Quaternion q_measure;
    q_measure.w = msg->pose.pose.orientation.w;
    q_measure.x = msg->pose.pose.orientation.x;
    q_measure.y = msg->pose.pose.orientation.y;
    q_measure.z = msg->pose.pose.orientation.z;
    Quaternion q_measured = body_axis_z(q_measure, 3.14159265358);

    odom_pos[3] = q_measured.x;
    odom_pos[4] = q_measured.y;
    odom_pos[5] = q_measured.z;
    odom_pos[6] = q_measured.w;
}

void getcloud_air(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr lasercloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;

    // geometry_msgs::TransformStamped base2map;
    // try{
    //     base2map = tfBuffer.lookupTransform("world", "body", ros::Time(0));
    // }catch (tf2::TransformException &ex){
    //     ROS_WARN("PCL2Process Get TF ERROR!");
    //     return;
    // }

    pcl::fromROSMsg(*laserCloudMsg, *lasercloud);

    Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();
    Eigen::Vector3f translation_eigen(odom_pos[0], odom_pos[1], odom_pos[2]);
    Eigen::Quaternionf eigen_rotation(odom_pos[6],odom_pos[3],odom_pos[4],odom_pos[5]);
    transform_eigen.translation() = translation_eigen;
    transform_eigen.linear() = eigen_rotation.toRotationMatrix();

    pcl::transformPointCloud(*lasercloud, *pcl2cloud, transform_eigen);

//    current_x = base2map.transform.translation.x;
//    current_y = base2map.transform.translation.y;
//    current_z = base2map.transform.translation.z;
//
//    if ((pcl2cloud->points.size()) == 0){
//        return;
//    }else{
//        long point_num = 0;
//        for (long i = 0; i < pcl2cloud->points.size(); i = i + 1){
//            if (pcl2cloud->points[i].x - current_x < 0.25
//                  and pcl2cloud->points[i].x - current_x > -0.25
//                  and pcl2cloud->points[i].y - current_y < 0.25
//                  and pcl2cloud->points[i].y - current_y > -0.25){
//                continue;
//            }
//            if (pcl2cloud->points[i].z - current_z < 0.4 and pcl2cloud->points[i].z - current_z > -0.4)
//            {
//                pcl2cloud->points[i].z = 0.1;
//                pcl2cloud_out->points.push_back(pcl2cloud->points[i]);
//                point_num = point_num + 1;
//            }
//        }
//
//        pcl2cloud_out->width = point_num;
//        pcl2cloud_out->height = 1;
//        pcl2cloud_out->points.resize(pcl2cloud_out->width * pcl2cloud_out->height);

//        pcl::toROSMsg(*pcl2cloud_out, ROSPCL_output);
        pcl::toROSMsg(*pcl2cloud, ROSPCL_output);

        ROSPCL_output.header.frame_id = "world";
        ROSPCL_output.header.stamp = ros::Time::now();
        pcl_publisher.publish(ROSPCL_output);
//    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    auto subCloud = pnh.subscribe<sensor_msgs::PointCloud2>("/pointcloud2_in", 1, getcloud_air);
    ros::Subscriber pose_sub = pnh.subscribe("/odometry/filtered", 10, poseCallback);
    pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
    ros::spin();
    return 0;
}
