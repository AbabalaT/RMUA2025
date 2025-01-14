//
// Created by tc on 24-12-20.
//
#include "basic_control.hpp"
#include <cmath>

float rc_channel[6] = {0.0, 0.0, -1000.0, 0.0, 0.0, 0.0};
int ctrl_mode = 0;
int rc_mode = 0;

typedef struct {
    float w, x, y, z;
} Quaternion;

float mat_pid[4][4];	//R-P-Y-throttle
float angle_pid_mat[3][3];
double velocity_pid_mat[4][4];

int init_waiting = 55;

float throttle_set = 0.0f;

float kalman_q = 5000.0f;
float kalman_r = 10000000.0f;

float u_real_roll = 0.0f;
float u_real_pitch = 0.0f;
float u_real_yaw = 0.0f;

float gyro_data[3], angle_data[3];

float w_yaw_world[3];
float w_yaw_body[3];

float motor_idle_speed = 0.0;
float motor_max_speed = 1000.0;

float pid_N = 0.35f;//离散pid微分低通滤波
float pid_N_v = 0.7f;

long imu_t;
long pose_t;
long pre_imu_sec, pre_imu_nsec, pre_pose_sec, pre_pose_nsec;

float kalman_roll(float measure){
	static float x;
	static float p;
	static float k;
	p = p + kalman_q;
	k = p / (p + kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

float kalman_pitch(float measure){
	static float x;
	static float p;
	static float k;
	p = p + kalman_q;
	k = p / (p + kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

float kalman_yaw(float measure){
	static float x;
	static float p;
	static float k;
	p = p + kalman_q;
	k = p / (p + kalman_r);
	x = x + k * (measure - x);
	p = (1.0f - k) * p;
	return x;	
}

void pid_init(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 27.0;
    mat_pid[0][2] = 8.5;
	mat_pid[0][3] = 4.0;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 27.0f;//697.6f;
	mat_pid[1][2] = 8.5;
	mat_pid[1][3] = 4.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 100.0f;//139.53f;
	mat_pid[2][2] = 60.0f;
	mat_pid[2][3] = 24.0;
	
	angle_pid_mat[0][0] = 1.2;
	angle_pid_mat[0][1] = 0.0001f;//0.00006;//232.55f;
	angle_pid_mat[0][2] = 0.045f;

	angle_pid_mat[1][0] = 1.2;
	angle_pid_mat[1][1] = 0.0001f;//0.00002f;//697.6f;
	angle_pid_mat[1][2] = 0.045f;
	
	angle_pid_mat[2][0] = 1.2;
	angle_pid_mat[2][1] = 0.0001f;//0.000045f;//139.53f;
	angle_pid_mat[2][2] = 0.045f;

//	velocity_pid_mat[1][0] = 0.0; //horizen
//	velocity_pid_mat[1][1] = 0.02f;//697.6f;
//	velocity_pid_mat[1][2] = 0.002;
//	velocity_pid_mat[1][3] = 0.00004;//0.00004;

    velocity_pid_mat[1][0] = 0.0; //horizen
	velocity_pid_mat[1][1] = 0.05f;//697.6f;
	velocity_pid_mat[1][2] = 0.08;
	velocity_pid_mat[1][3] = 0.024;//0.00004;

	velocity_pid_mat[2][0] = 0.0; //vertical
	velocity_pid_mat[2][1] = 0.075f;//139.53f;
	velocity_pid_mat[2][2] = 0.5f;
	velocity_pid_mat[2][3] = 0.015f;//0.015;
}

float pid_roll(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;

	error = target - real;
	error = target - real;
	sum = sum + error;

	if(sum > 10000.0f){
		sum = 10000.0;
	}
	if(sum < -10000.0f){
		sum = -10000.0;
	}
	if(error > 10.0f){
		sum = 0.0f;
	}
	if(error < -10.0f){
		sum = 0.0f;
	}
	if(throttle_set < 0.010f){
		sum = 0.0f;
	}
	if(init_waiting > 0){
          sum = 0.0f;
	}
	d_error = 0.0f - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;

	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1;
	d_out_1 = d_out;

	result = mat_pid[0][0]*target + mat_pid[0][1]*error + mat_pid[0][2]*sum*0.01 + mat_pid[0][3]*d_out / 0.01;
	return result;
}

float pid_pitch(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;

	error = target - real;
	sum = sum + error;

	if(sum > 10000.0f){
		sum = 10000.0;
	}
	if(sum < -10000.0f){
		sum = -10000.0;
	}
	if(error > 7.2f){
		sum = 0.0f;
	}
	if(init_waiting > 0){
		sum = 0.0f;
	}
	if(error < -7.2f){
		sum = 0.0f;
	}
	if(throttle_set < 0.010f){
		sum = 0.0f;
	}

	d_error = 0.0f - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;

	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1;
	d_out_1 = d_out;

	result = mat_pid[1][0]*target + mat_pid[1][1]*error + mat_pid[1][2]*sum*0.01 + mat_pid[1][3]*d_out / 0.01;
	return result;
}

float pid_yaw(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;

	error = target - real;
	sum = sum + error;

	if(sum > 10000.0f){
		sum = 10000.0;
	}
	if(init_waiting > 0){
		sum = 0.0f;
	}
	if(sum < -10000.0f){
		sum = -10000.0;
	}

	if(throttle_set < 0.010f){
		sum = 0.0f;
	}

	d_error = 0.0f - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;

	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1;
	d_out_1 = d_out;

	result = mat_pid[2][0]*target + mat_pid[2][1]*error + mat_pid[2][2]*sum*0.01 + mat_pid[2][3]*d_out / 0.01;
	return result;
}

float pid_angle_roll(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0f;
	}
	if(sum < -25000.0f){
		sum = -25000.0f;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 0.010f){
		sum = 0.0f;
	}
	if(ctrl_mode == 0){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[0][0]*error + angle_pid_mat[0][1] * 0.01 * sum + angle_pid_mat[0][2] / 0.01 * error_rate;
	return result;
}

float pid_angle_pitch(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0;
	}
	if(sum < -25000.0f){
		sum = -25000.0;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 0.010f){
		sum = 0.0f;
	}
	if(ctrl_mode == 0){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[1][0]*error + angle_pid_mat[1][1] * 0.01 * sum + angle_pid_mat[1][2] / 0.01 * error_rate;
	return result;
}

float pid_angle_yaw(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0f;
	}
	if(sum < -25000.0f){
		sum = -25000.0f;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 0.010f){
		sum = 0.0f;
	}
	if(ctrl_mode == 0){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[2][0]*error + angle_pid_mat[2][1] * 0.01 * sum + angle_pid_mat[2][2] / 0.01 *error_rate;
	return result;
}

float pid_vx(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;

	error = target - real;
	sum = sum + error;

	if(sum > 2000.0f){
		sum = 2000.0;
	}
	if(sum < -2000.0f){
		sum = -2000.0;
	}
	if(error > 50.0f){
		sum = 0.0f;
	}
	if(error < -50.0f){
		sum = 0.0f;
	}
	if(init_waiting > 0){
		sum = 0.0f;
	}
	d_error = 0.0 - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;

	d_out =  pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
	d_out_1 = d_out;

	result = velocity_pid_mat[1][0]*target + velocity_pid_mat[1][1]*(error + velocity_pid_mat[1][2]*sum*0.01 + velocity_pid_mat[1][3]*d_out / 0.01);
	return result;
}

float pid_vy(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;

	error = target - real;
	sum = sum + error;

	if(sum > 2000.0f){
		sum = 2000.0;
	}
	if(sum < -2000.0f){
		sum = -2000.0;
	}
	if(error > 50.0f){
		sum = 0.0f;
	}
	if(error < -50.0f){
		sum = 0.0f;
	}
	if(init_waiting > 0){
		sum = 0.0f;
	}
	d_error = 0.0 - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;

	d_out =  pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
	d_out_1 = d_out;

	result = velocity_pid_mat[1][0]*target + velocity_pid_mat[1][1]*(error + velocity_pid_mat[1][2]*sum*0.01 + velocity_pid_mat[1][3]*d_out / 0.01);
	return result;
}

float pid_vz(float target, float real){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;

	error = target - real;
	sum = sum + error;

	if(sum > 2000.0f){
		sum = 2000.0;
	}
	if(sum < -2000.0f){
		sum = -2000.0;
	}
	if(error > 50.0f){
		sum = 0.0f;
	}
	if(error < -50.0f){
		sum = 0.0f;
	}
	if(init_waiting > 0){
		sum = 0.0f;
	}
	d_error = 0.0 - real;
	error_rate = d_error - pre_error;
	pre_error = d_error;

	d_out =  pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
	d_out_1 = d_out;

	result = velocity_pid_mat[2][0]*target + velocity_pid_mat[2][1]*(error + velocity_pid_mat[2][2]*sum*0.01 + velocity_pid_mat[2][3]*d_out / 0.01);
	return result;
}

float output_roll;
float output_pitch;
float output_yaw;
float imu_roll;
float imu_pitch;
float imu_yaw;
float target_velocity_roll = 0.0f;
float target_velocity_pitch = 0.0f;
float target_velocity_yaw = 0.0f;

float target_angle_roll = 0.0f;
float target_angle_pitch = 0.0f;
float target_w_yaw = 0.0f;


Quaternion yaw_to_quaternion(double yaw) {
    Quaternion quaternion;
    quaternion.w = cos(yaw / 2);
    quaternion.x = 0;
    quaternion.y = 0;
    quaternion.z = sin(yaw / 2);
    return quaternion;
}

// Function to convert pitch angle (rotation around Y-axis) to quaternion
Quaternion pitch_to_quaternion(double pitch) {
    Quaternion quaternion;
    quaternion.w = cos(pitch / 2);
    quaternion.x = 0;
    quaternion.y = sin(pitch / 2);
    quaternion.z = 0;
    return quaternion;
}

Quaternion roll_to_quaternion(double roll) {
    Quaternion quaternion;
    quaternion.w = cos(roll / 2);
    quaternion.x = sin(roll / 2);
    quaternion.y = 0;
    quaternion.z = 0;
    return quaternion;
}

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

Quaternion quaternion_diff(Quaternion q1, Quaternion q2) {
		q1 = quaternion_conjugate(q1);
    Quaternion result = multiply_quaternion(&q2, &q1);
		if(result.w < 0.0f){
			result.w = -result.w;
			result.x = -result.x;
			result.y = -result.y;
			result.z = -result.z;
		}
    return result;
}

void quaternionToAngles(Quaternion q, float *roll, float *pitch, float *yaw) {
	float we = q.w;
	if(we > 0.999999f){
		we = 0.999999f;
	}
	if(we < -0.999999f){
		we = -0.999999f;
	}
	float theta = 2.0f * acosf(we);
	float ne = sqrtf(1.0f - we * we);
	float nx = q.x / ne;
	float ny = q.y / ne;
	float nz = q.z / ne;
	*pitch = ny * theta;
	*roll = nx * theta;
	*yaw = nz * theta;
}

float euler_angle[3];
float error_angle[3];
float error_body[3];
Quaternion target_quaternion;
Quaternion measure_quaternion;
float target_yaw = 0.0f;

void World_to_Body(float *vector_e, float *vector_v,Quaternion Qin)
{
	float C11,C12,C13;
	float C21,C22,C23;
	float C31,C32,C33;

	float Q[4];

	Q[0] =  Qin.w;
	Q[1] = -Qin.x;
	Q[2] = -Qin.y;
	Q[3] = -Qin.z;


	C11 = Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3];
	C12 = 2.0f*(Q[1]*Q[2] - Q[0]*Q[3]);
	C13 = 2.0f*(Q[1]*Q[3] + Q[0]*Q[2]);

	C21 = 2.0f*(Q[1]*Q[2] + Q[0]*Q[3]);
	C22 = Q[0]*Q[0] - Q[1]*Q[1] + Q[2]*Q[2] - Q[3]*Q[3];
	C23 = 2.0f*(Q[2]*Q[3] - Q[0]*Q[1]);

	C31 = 2.0f*(Q[1]*Q[3] - Q[0]*Q[2]);
	C32 = 2.0f*(Q[2]*Q[3] + Q[0]*Q[1]);
	C33 = Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3];

	vector_v[0] = C11*vector_e[0] + C12*vector_e[1] + C13*vector_e[2];
	vector_v[1] = C21*vector_e[0] + C22*vector_e[1] + C23*vector_e[2];
	vector_v[2] = C31*vector_e[0] + C32*vector_e[1] + C33*vector_e[2];

}

int main(int argc, char** argv){
  ros::init(argc, argv, "basic_control");
  ros::NodeHandle node_handle;
  rc_mode = 0;
  ctrl_mode = 2;
  pid_init();
  BasicControl control(&node_handle);
  return 0;
}

BasicControl::BasicControl(ros::NodeHandle *nh){
	pwm_cmd.rotorPWM0 = 0.1;
	pwm_cmd.rotorPWM1 = 0.1;
	pwm_cmd.rotorPWM2 = 0.1;
	pwm_cmd.rotorPWM3 = 0.1;

  rc_channel1_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc1", 1, std::bind(&BasicControl::channel1_callback, this, std::placeholders::_1));
  rc_channel2_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc2", 1, std::bind(&BasicControl::channel2_callback, this, std::placeholders::_1));
  rc_channel3_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc3", 1, std::bind(&BasicControl::channel3_callback, this, std::placeholders::_1));
  rc_channel4_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc4", 1, std::bind(&BasicControl::channel4_callback, this, std::placeholders::_1));
  rc_channel5_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc5", 1, std::bind(&BasicControl::channel5_callback, this, std::placeholders::_1));
  rc_channel6_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc6", 1, std::bind(&BasicControl::channel6_callback, this, std::placeholders::_1));
  pose_suber = nh->subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, std::bind(&BasicControl::poseCallback, this, std::placeholders::_1));//TODO(change to estimated pose)
  nh->setParam("/custom_debug/rc_mode", 3);//TODO(change to 0 later)
  imu_suber = nh->subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu", 1, std::bind(&BasicControl::imuCallback, this, std::placeholders::_1));
  pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);

  	rate_x_target_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/target_rate_x", 10);
	rate_y_target_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/target_rate_y", 10);
	rate_z_target_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/target_rate_z", 10);
	rate_x_real_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/real_rate_x", 10);
	rate_y_real_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/real_rate_y", 10);
	rate_z_real_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/real_rate_z", 10);

  rc_mode_timer = nh->createTimer(ros::Duration(0.2), std::bind(&BasicControl::rc_mode_check_callback, this, std::placeholders::_1));

  ros::spin();
}

BasicControl::~BasicControl(){}

float target_vel_x;
float target_vel_y;
float target_vel_z;

float hover_throttle = 0.175;

void BasicControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg){

  	float target_vel_body[3];

	if(ctrl_mode == 2 and rc_mode == 1){
		target_vel_body[1] = rc_channel[0] / -50.0f;
		target_vel_body[0] = rc_channel[1] / 50.0f;
		target_w_yaw = rc_channel[3] * -0.002341f;
		target_vel_body[2] = rc_channel[2] / 100.0f;
	}

    float body_measure_vel[3];
    body_measure_vel[0] = msg->twist.twist.linear.x;
    body_measure_vel[1] = msg->twist.twist.linear.y;
    body_measure_vel[2] = msg->twist.twist.linear.z;

	Quaternion odom_pose;
	odom_pose.w = msg->pose.pose.orientation.w;
	odom_pose.x = msg->pose.pose.orientation.x;
	odom_pose.y = msg->pose.pose.orientation.y;
	odom_pose.z = msg->pose.pose.orientation.z;

    Quaternion body_to_world = quaternion_conjugate(odom_pose);
    float world_measure_vel[3];
	World_to_Body(body_measure_vel, world_measure_vel, body_to_world);

	float measure_yaw = 0.0f;
	measure_yaw = atan2(2.0 * (odom_pose.w * odom_pose.z + odom_pose.x * odom_pose.y),
		1.0 - 2.0 * (odom_pose.y * odom_pose.y + odom_pose.z * odom_pose.z));
	Quaternion yaw_quaternion = yaw_to_quaternion(measure_yaw);
    Quaternion body_to_yaw = quaternion_conjugate(yaw_quaternion);
    float world_target_vel[3];
	World_to_Body(target_vel_body, world_target_vel, body_to_yaw);

    float world_force[3];
    float body_force[3];

	world_force[0] = pid_vx(world_target_vel[0], world_measure_vel[0]);
    world_force[1] = pid_vy(world_target_vel[1], world_measure_vel[1]);
	world_force[2] = pid_vz(world_target_vel[2], world_measure_vel[2]);

    world_force[2] = world_force[2] + hover_throttle;
    if(world_force[2] < 0.05f){
      world_force[2] = 0.05f;
    }

    World_to_Body(world_force, body_force, yaw_quaternion);
    throttle_set = sqrt(body_force[0] * body_force[0] + body_force[1] * body_force[1] + body_force[2] * body_force[2]);
	float norm_body_force[3];
    float up_vector[3];
//    norm_body_force[0] = body_force[0] / throttle_set;
//    norm_body_force[1] = body_force[1] / throttle_set;
    norm_body_force[2] = body_force[2] / throttle_set;
//    std::cout << "norm_body_force: " << norm_body_force[2] << std::endl;
    float theta = acos(norm_body_force[2]);
//    std::cout << "theta = " << theta << std::endl;
    float sin_half_theta = sin(theta/2.0f);
    float cos_half_theta = cos(theta/2.0f);
    float norm_xy = sqrt(body_force[0] * body_force[0] + body_force[1] * body_force[1]);
//    Quaternion target_tilt;
    target_quaternion.w = cos_half_theta;
    target_quaternion.y = -sin_half_theta * body_force[0] / norm_xy;
    target_quaternion.x = -sin_half_theta * body_force[1] / norm_xy;
    target_quaternion.z = 0.0f;

//    target_angle_roll = -atan2(body_force[1], body_force[2]);
//    target_angle_pitch = -atan2(body_force[0], body_force[2]);
//    target_angle_pitch = -atan2(body_force[1], body_force[2]);
//    target_angle_roll = -asin(body_force[0] / throttle_set);
//
//    if(throttle_set > 1.0){
//      throttle_set = 1.0;
//    }
//    if(target_angle_roll > 1.2){
//      target_angle_roll = 1.2;
//    }
//    if(target_angle_roll < -1.2){
//      target_angle_roll = -1.2;
//    }
//    if(target_angle_pitch > 1.2){
//      target_angle_pitch = 1.2;
//    }
//    if(target_angle_pitch < -1.2){
//      target_angle_pitch = -1.2;
//    }
//    std::cout<<ros::Time::now()<<std::endl;
}

void BasicControl::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    measure_quaternion.w = msg->orientation.w;
    measure_quaternion.x = msg->orientation.x;
    measure_quaternion.y = msg->orientation.y;
   	measure_quaternion.z = msg->orientation.z;
	float measure_yaw = 0.0f;
    measure_yaw = atan2(2.0 * (measure_quaternion.w * measure_quaternion.z + measure_quaternion.x * measure_quaternion.y),
		1.0 - 2.0 * (measure_quaternion.y * measure_quaternion.y + measure_quaternion.z * measure_quaternion.z));
//    measure_yaw = measure_yaw * 180.0 / 3.14159265359;
//    std::cout << "measure_yaw: " << measure_yaw << std::endl;

    Quaternion de_yaw_quaternion = yaw_to_quaternion(-measure_yaw);
    Quaternion de_yaw_ahrs = multiply_quaternion(&de_yaw_quaternion, &measure_quaternion);


	if(ctrl_mode == 1 and rc_mode == 1){
         target_angle_roll = rc_channel[0] * 1.5e-3;
         target_angle_pitch = rc_channel[1] * -1.5e-3;
         target_w_yaw = rc_channel[3] * -0.002341f;
         throttle_set = (rc_channel[2] / 2.0 + 500.0)/1000.0;

         Quaternion temp_quaternion;
    target_quaternion.w = 1.0f;
	target_quaternion.x = 0.0f;
	target_quaternion.y = 0.0f;
	target_quaternion.z = 0.0f;

    temp_quaternion = pitch_to_quaternion(target_angle_pitch);
    target_quaternion = multiply_quaternion(&temp_quaternion, &target_quaternion);
	temp_quaternion = roll_to_quaternion(target_angle_roll);
	target_quaternion = multiply_quaternion(&target_quaternion, &temp_quaternion);
	}
//	std::cout << target_quaternion.w << " " << target_quaternion.x << " " << target_quaternion.y << " " << target_quaternion.z << std::endl;

    Quaternion diff_quaternion = quaternion_diff(de_yaw_ahrs, target_quaternion);

	quaternionToAngles(diff_quaternion, &error_angle[0], &error_angle[1], &error_angle[2]);
    if(isnan(error_angle[0])){
		error_angle[0] = 0.0f;
	}
	if(isnan(error_angle[1])){
		error_angle[1] = 0.0f;
	}
	if(isnan(error_angle[2])){
		error_angle[2] = 0.0f;
	}
	World_to_Body(error_angle, error_body, de_yaw_ahrs);
    w_yaw_world[0] = 0.0f;
	w_yaw_world[1] = 0.0f;
	w_yaw_world[2] = target_w_yaw;
    World_to_Body(w_yaw_world, w_yaw_body, measure_quaternion);


    target_velocity_roll = pid_angle_roll(error_body[0]) + w_yaw_body[0];
    target_velocity_pitch = -pid_angle_pitch(error_body[1]) - w_yaw_body[1];
	target_velocity_yaw = pid_angle_yaw(error_body[2]) + w_yaw_body[2];

	if(target_velocity_pitch > 3.0f){
		target_velocity_pitch = 3.0f;
	}
	if(target_velocity_pitch < -3.0f){
		target_velocity_pitch = -3.0f;
	}
	if(target_velocity_roll > 3.0f){
		target_velocity_roll = 3.0f;
	}
	if(target_velocity_roll < -3.0f){
		target_velocity_roll = -3.0f;
	}
	if(target_velocity_yaw > 3.0f){
		target_velocity_yaw = 3.0f;
	}
	if(target_velocity_yaw < -3.0f){
		target_velocity_yaw = -3.0f;
	}

	gyro_data[0] = msg->angular_velocity.x;
    gyro_data[1] = msg->angular_velocity.y;
    gyro_data[2] = msg->angular_velocity.z;

    if(ctrl_mode == 0 and rc_mode == 1){
		target_velocity_roll = rc_channel[0] * 0.004;
		target_velocity_pitch = rc_channel[1] * 0.004;
		target_velocity_yaw = rc_channel[3] * -0.004;
        throttle_set = (rc_channel[2] / 2.0 + 500.0)/1000.0;
    }

    imu_roll = gyro_data[0];
	imu_pitch = -gyro_data[1];
	imu_yaw = -gyro_data[2];
	float roll_in = kalman_roll(imu_roll);
	float pitch_in = kalman_pitch(imu_pitch);
	float yaw_in = kalman_yaw(imu_yaw);
//    std::cout << "roll_in: " << roll_in << std::endl;
	output_roll = pid_roll(target_velocity_roll, roll_in) / 500.0;
	output_pitch = pid_pitch(target_velocity_pitch, pitch_in) / 500.0;
	output_yaw = pid_yaw(target_velocity_yaw, yaw_in) / 500.0;

	float front_left_speed 	= 0.0 + output_roll - output_pitch + output_yaw + throttle_set;
    float front_right_speed = 0.0 - output_roll - output_pitch - output_yaw + throttle_set;
    float rear_left_speed 	= 0.0 + output_roll + output_pitch - output_yaw + throttle_set;
    float rear_right_speed 	= 0.0 - output_roll + output_pitch + output_yaw + throttle_set;

    if(front_left_speed < motor_idle_speed){
      front_left_speed = motor_idle_speed;
    }
    if(front_right_speed < motor_idle_speed){
      front_right_speed = motor_idle_speed;
    }
    if(rear_left_speed < motor_idle_speed){
      rear_left_speed = motor_idle_speed;
    }
    if(rear_right_speed < motor_idle_speed){
      rear_right_speed = motor_idle_speed;
    }
    if(front_left_speed > motor_max_speed){
      front_left_speed = motor_max_speed;
    }
    if(front_right_speed > motor_max_speed){
      front_right_speed = motor_max_speed;
    }
    if(rear_left_speed > motor_max_speed){
      rear_left_speed = motor_max_speed;
    }
    if(rear_right_speed > motor_max_speed){
      rear_right_speed = motor_max_speed;
    }

	pwm_cmd.rotorPWM0 = front_right_speed;
	pwm_cmd.rotorPWM1 = rear_left_speed;
	pwm_cmd.rotorPWM2 = front_left_speed;
	pwm_cmd.rotorPWM3 = rear_right_speed;
    if(init_waiting > 0){
    	ROS_INFO("UKF Waiting...");
    }else{
		pwm_publisher.publish(pwm_cmd);
    }

//	std_msgs::Float32 rate_msg;
//	rate_msg.data = roll_in;
//	rate_x_real_publisher.publish(rate_msg);
//    rate_msg.data = pitch_in;
//    rate_y_real_publisher.publish(rate_msg);
//    rate_msg.data = yaw_in;
//    rate_z_real_publisher.publish(rate_msg);
//    rate_msg.data = target_velocity_roll;
//    rate_x_target_publisher.publish(rate_msg);
//    rate_msg.data = target_velocity_pitch;
//    rate_y_target_publisher.publish(rate_msg);
//    rate_msg.data = target_velocity_yaw;
//    rate_z_target_publisher.publish(rate_msg);
}

void BasicControl::channel1_callback(const std_msgs::Float32::ConstPtr& msg){
  rc_channel[0] = msg->data;
}
void BasicControl::channel2_callback(const std_msgs::Float32::ConstPtr& msg){
  rc_channel[1] = msg->data;
}
void BasicControl::channel3_callback(const std_msgs::Float32::ConstPtr& msg){
  rc_channel[2] = msg->data;
}
void BasicControl::channel4_callback(const std_msgs::Float32::ConstPtr& msg){
  rc_channel[3] = msg->data;
}
void BasicControl::channel5_callback(const std_msgs::Float32::ConstPtr& msg){
  rc_channel[4] = msg->data;
}
void BasicControl::channel6_callback(const std_msgs::Float32::ConstPtr& msg){
  rc_channel[5] = msg->data;
}

void BasicControl::rc_mode_check_callback(const ros::TimerEvent& event){
  if(init_waiting > 0){
    init_waiting--;
  }
  int got_mode;
  ros::NodeHandle nh;
  nh.getParam("/custom_debug/rc_mode", got_mode);
  if(got_mode == 0){
    ctrl_mode = 2;
    rc_mode = 0;
  }else{
    rc_mode = 1;
    switch(got_mode){
      case 1:
        ctrl_mode = 0;
        break;
      case 2:
        ctrl_mode = 1;
        break;
      case 3:
       	ctrl_mode = 2;
        break;
      default:
        ctrl_mode = 1;
        break;
    }
  }
//  std::cout<<ctrl_mode<<" "<<rc_mode <<std::endl;
}
