//
// Created by bismarck on 12/11/22.
//

#include <cmath>
#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Dense>

tf2::Quaternion ned2enu_quat;

float real_world_v_x, real_world_v_y, real_world_v_z;
float real_world_acc_x, real_world_acc_y, real_world_acc_z;
float real_world_pre_v_x, real_world_pre_v_y, real_world_pre_v_z;
float real_world_pre_x, real_world_pre_y, real_world_pre_z;

pcl::PointCloud<pcl::PointXYZ>::Ptr route_cloud(new pcl::PointCloud<pcl::PointXYZ>);

int initialized = 0;

int start_seq = 40;

ros::Time imu_stamp;
ros::Time gps_stamp;

tf2_ros::Buffer tfBuffer;

ros::Publisher imu_now_pub;
ros::Publisher odom_pub;
ros::Publisher lio_pub;
ros::Publisher real_pose_pub;
ros::Publisher real_map_pub;
ros::Publisher pcl_pub;
ros::Publisher ahrs_pub;
ros::Publisher uwb_map_pub;
ros::Publisher acc_pub;
ros::Publisher sat_odom_pub;

std::ofstream outFile;

float init_pose_x, init_pose_y, init_pose_z;

typedef struct
{
    float w, x, y, z;
} Quaternion;

float mahony_quaternion[4];

class KalmanFilter {
public:
    KalmanFilter(double process_noise, double measurement_noise, double dt = 0.01) {
        this->dt = dt;

        // Initialize state vector [position, velocity]
        x = Eigen::Vector2d(0.0, 0.0);  // [position, velocity]

        // Initial covariance matrix (large initial uncertainty in position and velocity)
        P = Eigen::Matrix2d::Identity() * 100;

        // State transition matrix A (constant velocity model)
        A << 1, dt,
             0, 1;

        // Control input matrix B (acceleration affects position and velocity)
        B << 0.5 * dt * dt,
             dt;

        // Measurement matrix H (we only measure position)
        H = Eigen::MatrixXd(1,2);
        H << 1, 0;

        // Measurement noise covariance (position measurement noise < 0.1m)
        R = Eigen::MatrixXd(1, 1);

        R(0, 0) = 0.1 * 0.1;  // Position noise: 0.1m squared

        // Process noise covariance (based on real acceleration < 10 m/s^2 and max velocity 30m/s)
        Q = Eigen::Matrix2d::Zero();
        Q(0, 0) = 10 * 10;   // Position noise (in meters)
        Q(0, 1) = 10;        // Cross-term for position and velocity
        Q(1, 0) = 10;        // Cross-term for position and velocity
        Q(1, 1) = 10;        // Velocity noise (in meters per second)
    }

    // Predict step of the Kalman filter
    void predict(double acceleration) {
        // Predict the next state
        x = A * x + B * acceleration;  // x = A * x + B * u

        // Update the covariance
        P = A * P * A.transpose() + Q;
    }

    // Update step of the Kalman filter (using position measurement)
    void update(double position_measurement) {
        // Measurement residual
        Eigen::VectorXd y = Eigen::VectorXd(1);

        y(0) = position_measurement - x.x();

        // Measurement uncertainty
        Eigen::MatrixXd S = H * P * H.transpose() + R;

        // Kalman gain
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        // Update the state estimate
        x = x + K * y;

        // Update the covariance estimate
        P = P - K * H * P;
    }

    // Get the current state (position, velocity)
    Eigen::Vector2d getState() const {
        return x;
    }

private:
    double dt;        // Time step (100Hz -> 0.01s)
    Eigen::Vector2d x;      // State vector [position, velocity]
    Eigen::Matrix2d P;      // Covariance matrix
    Eigen::Matrix2d A;      // State transition matrix
    Eigen::Vector2d B;      // Control input matrix
    Eigen::MatrixXd H;      // Measurement matrix
    Eigen::MatrixXd Q;      // Process noise covariance
    Eigen::MatrixXd R;      // Measurement noise covariance
};

std::vector<KalmanFilter> kalman_filters;

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
    rotation.x = 0.0f;
    rotation.y = 0.0f;
    rotation.z = sin(theta / 2.0f);

    Quaternion result = multiply_quaternion(&measure, &rotation);
    return result;
}

void InitialPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    init_pose_x = msg->pose.position.y;
    init_pose_y = msg->pose.position.x;
    init_pose_z = -msg->pose.position.z - 0.3f;

    init_quat.x = msg->pose.orientation.y;
    init_quat.y = msg->pose.orientation.x;
    init_quat.z = -msg->pose.orientation.z;
    init_quat.w = msg->pose.orientation.w;
    world_quat_no_rotation = init_quat;
    Quaternion quat = body_axis_z(init_quat, 3.14159265359f / 2.0f);
    init_quat = quat;
}

float rc_channel[6];
float pre_rc_channel[6];

void channel6_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[5] = msg->data;
}

void RealPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";

    for (int i = 0; i < 35; i++)
    {
        odom_msg.pose.covariance[0] = 0.0;
    }

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
    //odom_pub.publish(odom_msg);//use real to debug
    odom_msg.header.frame_id = "map_odom";
    real_map_pub.publish(odom_msg);

    pcl::PointXYZ new_point;
    new_point.x = odom_msg.pose.pose.position.x;
    new_point.y = odom_msg.pose.pose.position.y;
    new_point.z = odom_msg.pose.pose.position.z;

    // route_cloud->points.push_back(new_point);

    if(rc_channel[5] < -100){
      outFile << new_point.x << ", " << new_point.y << ", " << new_point.z << std::endl;
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gps_stamp = msg->header.stamp;
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = msg->pose.position.y;
    odom_msg.pose.pose.position.y = msg->pose.position.x;
    odom_msg.pose.pose.position.z = -msg->pose.position.z;

    Quaternion q_measure;

    q_measure.w = msg->pose.orientation.w;
    q_measure.x = msg->pose.orientation.y;
    q_measure.y = msg->pose.orientation.x;
    q_measure.z = -msg->pose.orientation.z;

    Quaternion q_measured = body_axis_z(q_measure, 3.14159265359 / 2.0f);

    odom_msg.pose.pose.orientation.x = world_quat.x;
    odom_msg.pose.pose.orientation.y = world_quat.y;
    odom_msg.pose.pose.orientation.z = world_quat.z;
    odom_msg.pose.pose.orientation.w = world_quat.w;

    double conv_1 = 0.2f;
    double conv_2 = 0.01f;

    for (int i = 0; i < 35; i++)
    {
        odom_msg.pose.covariance[0] = 0.0;
    }

    odom_msg.pose.covariance[0] = conv_1;
    odom_msg.pose.covariance[7] = conv_1;
    odom_msg.pose.covariance[14] = conv_1;

    odom_msg.pose.covariance[21] = conv_2;
    odom_msg.pose.covariance[28] = conv_2;
    odom_msg.pose.covariance[35] = conv_2;

    odom_pub.publish(odom_msg);

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[14] = 0.01;

    odom_msg.pose.covariance[21] = 0.0001;
    odom_msg.pose.covariance[28] = 0.0001;
    odom_msg.pose.covariance[35] = 0.0001;

    odom_msg.header.frame_id = "map_odom";
    uwb_map_pub.publish(odom_msg);

    kalman_filters[0].update(odom_msg.pose.pose.position.x);
    kalman_filters[1].update(odom_msg.pose.pose.position.y);
    kalman_filters[2].update(odom_msg.pose.pose.position.z);
}

double zero[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

int imu_cnt = -99;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_stamp = ros::Time::now();

    sensor_msgs::Imu new_msg = *msg;
    new_msg.header.frame_id = "base_link";

    Quaternion q_measure;
    q_measure.w = new_msg.orientation.w;
    q_measure.x = new_msg.orientation.y;
    q_measure.y = new_msg.orientation.x;
    q_measure.z = -new_msg.orientation.z;

    Quaternion q_measure_world = multiply_quaternion(&world_quat_no_rotation, &q_measure);
    world_quat = body_axis_z(q_measure_world, 3.14159265359 / 2.0f);

    new_msg.orientation.x = world_quat.x;
    new_msg.orientation.y = world_quat.y;
    new_msg.orientation.z = world_quat.z;
    new_msg.orientation.w = world_quat.w;

    static double conv_11 = 0.0;
    static double conv_12 = 0.0;
    static double conv_13 = 0.0;
    static double conv_21 = 0.0;
    static double conv_22 = 0.0;
    static double conv_23 = 0.0;

    static double conv_3 = 0.0;
    static double conv_4 = 0.0;

    static double easy_cnt = 0.0;

    if (imu_cnt > 300)
    {
        new_msg.angular_velocity_covariance[0] = conv_21;
        new_msg.angular_velocity_covariance[4] = conv_22;
        new_msg.angular_velocity_covariance[8] = conv_23;

        new_msg.angular_velocity_covariance[1] = conv_3;
        new_msg.angular_velocity_covariance[2] = conv_3;
        new_msg.angular_velocity_covariance[3] = conv_3;
        new_msg.angular_velocity_covariance[5] = conv_3;
        new_msg.angular_velocity_covariance[6] = conv_3;
        new_msg.angular_velocity_covariance[7] = conv_3;

        new_msg.linear_acceleration_covariance[0] = conv_11;
        new_msg.linear_acceleration_covariance[4] = conv_12;
        new_msg.linear_acceleration_covariance[8] = conv_13;

        new_msg.linear_acceleration_covariance[1] = conv_4;
        new_msg.linear_acceleration_covariance[2] = conv_4;
        new_msg.linear_acceleration_covariance[3] = conv_4;
        new_msg.linear_acceleration_covariance[5] = conv_4;
        new_msg.linear_acceleration_covariance[6] = conv_4;
        new_msg.linear_acceleration_covariance[7] = conv_4;
    }
    //get world acc
    tf2::Quaternion world_to_body_quat(new_msg.orientation.x, new_msg.orientation.y, new_msg.orientation.z,
                                       new_msg.orientation.w);

    tf2::Quaternion body_to_world_quat = world_to_body_quat.inverse();
    tf2::Vector3 acc_body(new_msg.linear_acceleration.x, -new_msg.linear_acceleration.y, -new_msg.linear_acceleration.z);
    tf2::Vector3 acc_world = tf2::quatRotate(body_to_world_quat, acc_body);

    float acc_array[3] = {
        static_cast<float>(acc_world.x()), static_cast<float>(acc_world.y()), static_cast<float>(acc_world.z())
    };

    if (imu_cnt > 0)
    {
        acc_world.setZ(acc_world.z() - 9.806);
    }//clear gravity

    acc_body = tf2::quatRotate(world_to_body_quat, acc_world);

    new_msg.linear_acceleration.x = acc_body.x();
    new_msg.linear_acceleration.y = acc_body.y();
    new_msg.linear_acceleration.z = acc_body.z();

    new_msg.angular_velocity.y = -new_msg.angular_velocity.y;
    new_msg.angular_velocity.z = -new_msg.angular_velocity.z;

    if (imu_cnt <= 0)
    {
        std::cout<<"imu_cnt"<<imu_cnt<<std::endl;
        zero[6] = zero[6] + sqrt(acc_body.x()*acc_body.x()+acc_body.y()*acc_body.y()+acc_body.z()*acc_body.z());
        easy_cnt = easy_cnt + 1.0;
        if (imu_cnt == 0)
        {
            zero[6] /= easy_cnt;
            std::cout <<"estimated G:"<< zero[6] << std::endl;
            easy_cnt = 0.0;
        }
        imu_cnt = imu_cnt + 1;
        return;
    }//gravity estimate

    if (imu_cnt < 301)
    {
        zero[0] += new_msg.linear_acceleration.x;
        zero[1] += new_msg.linear_acceleration.y;
        zero[2] += new_msg.linear_acceleration.z;
        zero[3] += new_msg.angular_velocity.x;
        zero[4] += new_msg.angular_velocity.y;
        zero[5] += new_msg.angular_velocity.z;

        conv_11 += new_msg.linear_acceleration.x * new_msg.linear_acceleration.x;
        conv_12 += new_msg.linear_acceleration.y * new_msg.linear_acceleration.y;
        conv_13 += new_msg.linear_acceleration.z * new_msg.linear_acceleration.z;

        conv_21 += new_msg.angular_velocity.x * new_msg.angular_velocity.x;
        conv_22 += new_msg.angular_velocity.y * new_msg.angular_velocity.y;
        conv_23 += new_msg.angular_velocity.z * new_msg.angular_velocity.z;

        easy_cnt = easy_cnt + 1.0;

        if (imu_cnt == 300)
        {
            zero[0] = zero[0] / -easy_cnt;
            zero[1] = zero[1] / -easy_cnt;
            zero[2] = zero[2] / -easy_cnt;
            zero[3] = zero[3] / -easy_cnt;
            zero[4] = zero[4] / -easy_cnt;
            zero[5] = zero[5] / -easy_cnt;

            conv_11 /= easy_cnt;
            conv_12 /= easy_cnt;
            conv_13 /= easy_cnt;
            conv_21 /= easy_cnt;
            conv_22 /= easy_cnt;
            conv_23 /= easy_cnt;

            std::cout << zero[0] << " " << zero[1] << " " << zero[2] << " " << zero[3] << std::endl;
        }
        imu_cnt = imu_cnt + 1;
        return;
    }//zero estimate

    // new_msg.linear_acceleration.x += zero[0];
    // new_msg.linear_acceleration.y += zero[1];
    // new_msg.linear_acceleration.z += zero[2];
    // new_msg.angular_velocity.x += zero[3];
    // new_msg.angular_velocity.y += zero[4];
    // new_msg.angular_velocity.z += zero[5];

    imu_now_pub.publish(new_msg);

    std_msgs::Float32MultiArray acc_msg;
    acc_msg.data.assign(acc_array, acc_array + 3);
    acc_pub.publish(acc_msg);

    // std::cout<<"acc world z"<< acc_world.z()<<std::endl;

    kalman_filters[0].predict(acc_world.x());
    kalman_filters[1].predict(acc_world.y());
    kalman_filters[2].predict(acc_world.z());

    auto state_x = kalman_filters[0].getState();
    auto state_y = kalman_filters[1].getState();
    auto state_z = kalman_filters[2].getState();

    //std::cout << "x: " << state_x(0)<<", "<< state_x(1) << " y: " << state_y(0)<<", "<< state_y(1) << " z: " << state_z(0)<<", "<< state_z(1) << std::endl;
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = state_x(0);
    odom_msg.pose.pose.position.y = state_y(0);
    odom_msg.pose.pose.position.z = state_z(0);

    tf2::Vector3 vel_world(state_x(1), state_y(1), state_z(1));
    tf2::Vector3 vel_body = tf2::quatRotate(world_to_body_quat, vel_world);

    odom_msg.twist.twist.linear.x = vel_body.x();
    odom_msg.twist.twist.linear.y = vel_body.y();
    odom_msg.twist.twist.linear.z = vel_body.z();

    odom_msg.pose.pose.orientation.x = world_quat.x;
    odom_msg.pose.pose.orientation.y = world_quat.y;
    odom_msg.pose.pose.orientation.z = world_quat.z;
    odom_msg.pose.pose.orientation.w = world_quat.w;

    sat_odom_pub.publish(odom_msg);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped trans;

    trans.header.frame_id = "map";
    trans.child_frame_id = "base_link";
    trans.header.stamp = ros::Time::now();
    trans.transform.rotation.x = world_quat.x;
    trans.transform.rotation.y = world_quat.y;
    trans.transform.rotation.z = world_quat.z;
    trans.transform.rotation.w = world_quat.w;
    trans.transform.translation.x = odom_msg.pose.pose.position.x;
    trans.transform.translation.y = odom_msg.pose.pose.position.y;
    trans.transform.translation.z = odom_msg.pose.pose.position.z;
    br.sendTransform(trans);
}

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    sensor_msgs::PointCloud2 new_msg = *msg;
    new_msg.header.stamp = ros::Time::now();
    //    new_msg.header.frame_id = "body";
    pcl_pub.publish(new_msg);
}

void rviz_point_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    ROS_INFO("Clicked point: [%f, %f, %f]", msg->point.x, msg->point.y, msg->point.z);
    outFile << msg->point.x << ", " << msg->point.y << ", " << msg->point.z << std::endl;
}

void project2plane_callback(const ros::TimerEvent&)
{
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

    geometry_msgs::TransformStamped base2plane;
    try
    {
        base2plane = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        try
        {
            base2plane = tfBuffer.lookupTransform("map_odom", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("BaseLink Get TF ERROR!");
            return;
        }
    }
    base2plane.transform.translation.z = 0;
    base2plane.header.stamp = ros::Time::now();
    base2plane.child_frame_id = "plane_base_link";
    br.sendTransform(base2plane);
    tf2::Quaternion b2m{
        base2plane.transform.rotation.x, base2plane.transform.rotation.y,
        base2plane.transform.rotation.z, base2plane.transform.rotation.w
    };
    double roll = 0, pitch = 0, yaw = 0;
    tf2::Matrix3x3(b2m).getRPY(roll, pitch, yaw);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    base2plane.transform.rotation.x = q.x();
    base2plane.transform.rotation.y = q.y();
    base2plane.transform.rotation.z = q.z();
    base2plane.transform.rotation.w = q.w();
    base2plane.header.stamp = ros::Time::now();
    base2plane.child_frame_id = "nav_base_link";
    br.sendTransform(base2plane);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_process");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.01), project2plane_callback);

    ros::Subscriber imu_sub = pnh.subscribe("/airsim_node/drone_1/imu/imu", 10, imuCallback);
    ros::Subscriber pose_sub = pnh.subscribe("/airsim_node/drone_1/gps", 10, poseCallback);
    ros::Subscriber debug_pose = pnh.subscribe("/airsim_node/drone_1/debug/pose_gt", 10, RealPoseCallback);
    //ros::Subscriber pcl_sub = pnh.subscribe("/airsim_node/drone_1/lidar", 10, pclCallback);
    ros::Subscriber init_sub = pnh.subscribe("/airsim_node/initial_pose", 10, InitialPoseCallback);
    ros::Subscriber rviz_point_sub = pnh.subscribe("/clicked_point", 10, rviz_point_callback);
    ros::Subscriber rc6_sub = pnh.subscribe("/custom_debug/rc6", 10, channel6_callback);

    imu_now_pub = pnh.advertise<sensor_msgs::Imu>("/ekf/imu_now", 10);
    pcl_pub = pnh.advertise<sensor_msgs::PointCloud2>("/ekf/pcl", 10);
    odom_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/uwb", 10);
    real_pose_pub = pnh.advertise<nav_msgs::Odometry>("/debug/real_pose_odom", 10);
    real_map_pub = pnh.advertise<nav_msgs::Odometry>("/debug/real_map_odom", 10);
    lio_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/lio", 10);
    ahrs_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/ahrs", 10);
    uwb_map_pub = pnh.advertise<nav_msgs::Odometry>("/ekf/uwb2", 10);
    acc_pub = pnh.advertise<std_msgs::Float32MultiArray>("/exe/output_acc", 10);
    sat_odom_pub = pnh.advertise<nav_msgs::Odometry>("/odometry/filtered", 10);

    rc_channel[5] = 100.0;
    mahony_quaternion[0] = 1.0;
    mahony_quaternion[1] = 0.0;
    mahony_quaternion[2] = 0.0;
    mahony_quaternion[3] = 0.0;
    // pcl::io::loadPCDFile<pcl::PointXYZ>("/home/tc/route.pcd", *route_cloud);
    outFile.open("/home/tc/route_point.route", std::ios::app);
    std::cout<<"init filters"<<std::endl;
    kalman_filters.push_back(KalmanFilter(1e-5, 0.1));
    kalman_filters.push_back(KalmanFilter(1e-5, 0.1));
    kalman_filters.push_back(KalmanFilter(1e-5, 0.1));
    std::cout<<"init filters ok"<<std::endl;
    ros::spin();
    // route_cloud->width = route_cloud->points.size();
    // route_cloud->height = 1;
    // pcl::io::savePCDFileASCII("/home/tc/route.pcd", *route_cloud);
    outFile.close();
    return 0;
}
