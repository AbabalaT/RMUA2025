//
// Created by KevinTC.
//
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

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

struct PointInt
{
    int p_x, p_y, p_z;
};

float current_x = 0.0, current_y = 0.0, current_z = 0.0;

void getcloud_air(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr lasercloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl2cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 ROSPCL_output;

    geometry_msgs::TransformStamped base2map;
    try{
        base2map = tfBuffer.lookupTransform("map", "body", ros::Time(0));
    }catch (tf2::TransformException &ex){
        ROS_WARN("PCL2Process Get TF ERROR!");
        return;
    }

    pcl::fromROSMsg(*laserCloudMsg, *lasercloud);

    Eigen::Affine3f transform_eigen = Eigen::Affine3f::Identity();
    Eigen::Vector3f translation_eigen(base2map.transform.translation.x, base2map.transform.translation.y, base2map.transform.translation.z);
    Eigen::Quaternionf eigen_rotation(
        base2map.transform.rotation.w,
        base2map.transform.rotation.x,
        base2map.transform.rotation.y,
        base2map.transform.rotation.z);
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

        ROSPCL_output.header.frame_id = "map";
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
    pcl_publisher = pnh.advertise<sensor_msgs::PointCloud2>("/pointcloud2_out", 1);
    ros::spin();
    return 0;
}
