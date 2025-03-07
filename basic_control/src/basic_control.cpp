//
// Created by tc on 24-12-20.
//
#include "basic_control.hpp"
#include "mission_path.hpp"
#include <cmath>

float rc_channel[6] = {0.0, 0.0, -1000.0, 0.0, 0.0, 0.0};
int ctrl_mode = 0;
int rc_mode = 0;
float imu_angle[3];

typedef struct
{
    float w, x, y, z;
} Quaternion;

float mat_pid[4][4]; // R-P-Y-throttle
float angle_pid_mat[3][3];
double velocity_pid_mat[4][4];
double pos_pid_mat[4][4];

float target_world_pos[4];
float measure_world_pos[4];

float start_pose[3];
float end_pose[3];

float start_by_pose[3];
float end_by_pose[3];

float tf_cmd[11];
float no_g_acc[3];

int init_waiting = 200;

bool offboard_disable = false;

float throttle_set = 0.0f;

double kalman_q = 5000.0;
double kalman_r = 10000000.0;

float u_real_roll = 0.0f;
float u_real_pitch = 0.0f;
float u_real_yaw = 0.0f;

float gyro_data[3], angle_data[3];

float w_yaw_world[3];
float w_yaw_body[3];

float motor_idle_speed = 0.005;
float motor_max_speed = 0.995;

float pid_N = 0.5f; // 离散pid微分低通滤波
float pid_N_v = 0.9f;

long imu_t;
long pose_t;
long pre_imu_sec, pre_imu_nsec, pre_pose_sec, pre_pose_nsec;

bool force_strong_power_mode = false;
bool force_weak_power_mode = false;

bool weak_power_state;

double kalman_roll(float measure)
{
    static double x;
    static double p;
    static double k;
    p = p + kalman_q;
    k = p / (p + kalman_r);
    x = x + k * (measure - x);
    p = (1.0 - k) * p;
    return measure;
}

float kalman_pitch(float measure)
{
    static float x;
    static float p;
    static float k;
    p = p + kalman_q;
    k = p / (p + kalman_r);
    x = x + k * (measure - x);
    p = (1.0f - k) * p;
    return measure;
}

float kalman_yaw(float measure)
{
    static float x;
    static float p;
    static float k;
    p = p + kalman_q;
    k = p / (p + kalman_r);
    x = x + k * (measure - x);
    p = (1.0f - k) * p;
    return measure;
}

// Butterworth filter
#define PI 3.14159265358979323846

// 定义滤波器结构体
typedef struct
{
    // 滤波器系数
    double b0, b1, b2;
    double a1, a2;

    // 滤波器状态
    double x1, x2; // 输入状态
    double y1, y2; // 输出状态
} ButterworthFilter;

// 初始化巴特沃斯低通滤波器
void initButterworthFilter(ButterworthFilter* filter, double sampleRate, double cutoffFreq)
{
    double omega_c = 2.0 * PI * cutoffFreq / sampleRate;
    double alpha = sin(omega_c) / 2.0;

    // 计算滤波器系数
    double b0 = (1 - cos(omega_c)) / 2;
    double b1 = 1 - cos(omega_c);
    double b2 = (1 - cos(omega_c)) / 2;
    double a0 = 1 + alpha;
    double a1 = -2 * cos(omega_c);
    double a2 = 1 - alpha;

    // 归一化系数
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // 初始化状态
    filter->x1 = 0;
    filter->x2 = 0;
    filter->y1 = 0;
    filter->y2 = 0;
}

// 应用滤波器
double applyButterworthFilter(ButterworthFilter* filter, double input)
{
    // 计算输出
    double output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 -
        filter->a2 * filter->y2;

    // 更新状态
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

ButterworthFilter world_vel_x_filter, world_vel_y_filter, world_vel_z_filter;
ButterworthFilter world_pos_x_filter, world_pos_y_filter, world_pos_z_filter;

float point_distance(const float p1[], const float p2[])
{
    float dist = 0.0;
    for (int i = 0; i < 3; i++)
    {
        dist += (p2[i] - p1[i]) * (p2[i] - p1[i]);
    }
    return sqrt(dist);
}

void pid_init(void)
{
    mat_pid[0][0] = 0.0;
    mat_pid[0][1] = 7.0;
    mat_pid[0][2] = 48.0;
    mat_pid[0][3] = 0.04;

    mat_pid[1][0] = 0.0;
    mat_pid[1][1] = 7.0f;
    mat_pid[1][2] = 48.0;
    mat_pid[1][3] = 0.04;

    mat_pid[2][0] = 0.0;
    mat_pid[2][1] = 120.0f;
    mat_pid[2][2] = 0.0f;
    mat_pid[2][3] = 0.0;

    angle_pid_mat[0][0] = 4.2;
    angle_pid_mat[0][1] = 0.0f;
    angle_pid_mat[0][2] = 0.0f;

    angle_pid_mat[1][0] = 4.2;
    angle_pid_mat[1][1] = 0.0f;
    angle_pid_mat[1][2] = 0.0f;

    angle_pid_mat[2][0] = 4.2;
    angle_pid_mat[2][1] = 0.0f;
    angle_pid_mat[2][2] = 0.0f;

    velocity_pid_mat[1][0] = 0.0;
    velocity_pid_mat[1][1] = 0.04f;
    velocity_pid_mat[1][2] = 0.0;
    velocity_pid_mat[1][3] = 0.0001;

    velocity_pid_mat[2][0] = 0.0;
    velocity_pid_mat[2][1] = 0.04f;
    velocity_pid_mat[2][2] = 0.0f;
    velocity_pid_mat[2][3] = 0.0001f;

    pos_pid_mat[0][0] = 0.0;
    pos_pid_mat[0][1] = 0.6;
    pos_pid_mat[0][2] = 0.0;
    pos_pid_mat[0][3] = 0.1;

    pos_pid_mat[1][0] = 0.0;
    pos_pid_mat[1][1] = 0.6;
    pos_pid_mat[1][2] = 0.0;
    pos_pid_mat[1][3] = 0.1;

    pos_pid_mat[2][0] = 0.0;
    pos_pid_mat[2][1] = 0.6;
    pos_pid_mat[2][2] = 0.0;
    pos_pid_mat[2][3] = 0.1;

    pos_pid_mat[3][0] = 0.0;
    pos_pid_mat[3][1] = 6.0;
    pos_pid_mat[3][2] = 0.0;
    pos_pid_mat[3][3] = 0.0;
}

void pid_weak(void)
{
    mat_pid[0][0] = 0.0;
    mat_pid[0][1] = 30.0;
    mat_pid[0][2] = 0.0;
    mat_pid[0][3] = 0.00;

    mat_pid[1][0] = 0.0;
    mat_pid[1][1] = 30.0f;
    mat_pid[1][2] = 0.0;
    mat_pid[1][3] = 0.00;

    mat_pid[2][0] = 0.0;
    mat_pid[2][1] = 360.0f;
    mat_pid[2][2] = 0.0f;
    mat_pid[2][3] = 0.0;

    angle_pid_mat[0][0] = 4.2;
    angle_pid_mat[0][1] = 0.0f;
    angle_pid_mat[0][2] = 0.0f;

    angle_pid_mat[1][0] = 4.2;
    angle_pid_mat[1][1] = 0.0f;
    angle_pid_mat[1][2] = 0.0f;

    angle_pid_mat[2][0] = 4.2;
    angle_pid_mat[2][1] = 0.0f;
    angle_pid_mat[2][2] = 0.0f;

    velocity_pid_mat[1][0] = 0.0;
    velocity_pid_mat[1][1] = 0.12f;
    velocity_pid_mat[1][2] = 0.00;
    velocity_pid_mat[1][3] = 0.001;

    velocity_pid_mat[2][0] = 0.0;
    velocity_pid_mat[2][1] = 0.12f;
    velocity_pid_mat[2][2] = 0.00f;
    velocity_pid_mat[2][3] = 0.001f;

    pos_pid_mat[0][0] = 0.0;
    pos_pid_mat[0][1] = 0.6;
    pos_pid_mat[0][2] = 0.0;
    pos_pid_mat[0][3] = 0.01;

    pos_pid_mat[1][0] = 0.0;
    pos_pid_mat[1][1] = 0.6;
    pos_pid_mat[1][2] = 0.0;
    pos_pid_mat[1][3] = 0.01;

    pos_pid_mat[2][0] = 0.0;
    pos_pid_mat[2][1] = 0.6;
    pos_pid_mat[2][2] = 0.0;
    pos_pid_mat[2][3] = 0.01;

    pos_pid_mat[3][0] = 0.0;
    pos_pid_mat[3][1] = 2.0;
    pos_pid_mat[3][2] = 0.0;
    pos_pid_mat[3][3] = 0.0;
}

float pid_roll(float target, float real, float dt)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;
    sum = sum + error * dt;

    if (sum > 0.085f)
    {
        sum = 0.0;
    }
    if (sum < -0.085f)
    {
        sum = -0.0;
    }

    if (error > 8.0f)
    {
        sum = 0.0f;
    }
    if (error < -8.0f)
    {
        sum = 0.0f;
    }

    if (init_waiting > 0)
    {
        sum = 0.0f;
    }
    if (error < -7.2f)
    {
        sum = 0.0f;
    }
    if (throttle_set < 0.03f)
    {
        sum = 0.0f;
    }

    d_error = 0.0f - real;
    error_rate = (d_error - pre_error)/ dt;

    if (error_rate > 50.0f)
    {
        error_rate = 20.0f;
    }
    if (error_rate < -50.0f)
    {
        error_rate = -20.0;
    }

    pre_error = d_error;

    d_out = pid_N * error_rate + (1.0f - pid_N) * d_out_1;
    d_out_1 = d_out;

    static float max_i;
    static float max_d;

    if (sum > max_i)
    {
        max_i = sum;
    }
    if (error_rate > max_d)
    {
        max_d = error_rate;
    }
    result = mat_pid[0][0] * target + mat_pid[0][1] * error + mat_pid[0][2] * sum + mat_pid[0][3] * d_out ;
    return result;
}

float pid_pitch(float target, float real, float dt)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;
    sum = sum + error * dt;

    if (sum > 0.085f)
    {
        sum = 0.0;
    }
    if (sum < -0.085f)
    {
        sum = -0.0;
    }

    if (error > 8.0f)
    {
        sum = 0.0f;
    }
    if (error < -8.0f)
    {
        sum = 0.0f;
    }

    if (init_waiting > 0)
    {
        sum = 0.0f;
    }
    if (error < -7.2f)
    {
        sum = 0.0f;
    }
    if (throttle_set < 0.010f)
    {
        sum = 0.0f;
    }

    d_error = 0.0f - real;
    error_rate = (d_error - pre_error)/ dt;

    if (error_rate > 70.0f)
    {
        error_rate = 20.0f;
    }
    if (error_rate < -70.0f)
    {
        error_rate = -20.0;
    }

    pre_error = d_error;

    d_out = pid_N * error_rate + (1.0f - pid_N) * d_out_1;
    d_out_1 = d_out;

    result = mat_pid[1][0] * target + mat_pid[1][1] * error + mat_pid[1][2] * sum + mat_pid[1][3] * d_out;
    return result;
}

float pid_yaw(float target, float real, float dt)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    sum = sum + error * dt;

    if (sum > 0.085f)
    {
        sum = 0.0;
    }
    if (sum < -0.085f)
    {
        sum = -0.0;
    }

    if (error > 8.0f)
    {
        sum = 0.0f;
    }
    if (error < -8.0f)
    {
        sum = 0.0f;
    }

    if (throttle_set < 0.010f)
    {
        sum = 0.0f;
    }

    d_error = 0.0f - real;
    error_rate = (d_error - pre_error)/ dt;

    if (error_rate > 70.0f)
    {
        error_rate = 20.0f;
    }
    if (error_rate < -70.0f)
    {
        error_rate = -20.0;
    }

    pre_error = d_error;

    d_out = pid_N * error_rate + (1.0f - pid_N) * d_out_1;
    d_out_1 = d_out;

    result = mat_pid[2][0] * target + mat_pid[2][1] * error + mat_pid[2][2] * sum + mat_pid[2][3] * d_out;
    return result;
}

float pid_angle_roll(float error, float dt)
{
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;

    sum = sum + error * dt;

    if (sum > 0.25f)
    {
        sum = 0.25f;
    }
    if (sum < -0.25f)
    {
        sum = -0.25f;
    }

    if (error > 45.0f)
    {
        sum = 0.0f;
    }
    if (error < -45.0f)
    {
        sum = 0.0f;
    }

    if (throttle_set < 0.010f)
    {
        sum = 0.0f;
    }
    if (ctrl_mode == 0)
    {
        sum = 0.0f;
    }

    error_rate = (error - pre_error) / dt;
    pre_error = error;

    result = angle_pid_mat[0][0] * error + angle_pid_mat[0][1] * sum + angle_pid_mat[0][2] * error_rate;
    return result;
}

float pid_angle_pitch(float error, float dt)
{
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    sum = sum + error * dt;

    if (sum > 0.25f)
    {
        sum = 0.25f;
    }
    if (sum < -0.25f)
    {
        sum = -0.25f;
    }

    if (error > 45.0f)
    {
        sum = 0.0f;
    }
    if (error < -45.0f)
    {
        sum = 0.0f;
    }

    if (throttle_set < 0.010f)
    {
        sum = 0.0f;
    }
    if (ctrl_mode == 0)
    {
        sum = 0.0f;
    }

    error_rate = (error - pre_error) / dt;
    pre_error = error;

    result = angle_pid_mat[1][0] * error + angle_pid_mat[1][1] * sum + angle_pid_mat[1][2] * error_rate;
    return result;
}

float pid_angle_yaw(float error, float dt)
{
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;

    sum = sum + error * dt;

    if (sum > 0.25f)
    {
        sum = 0.25f;
    }
    if (sum < -0.25f)
    {
        sum = -0.25f;
    }

    if (error > 45.0f)
    {
        sum = 0.0f;
    }
    if (error < -45.0f)
    {
        sum = 0.0f;
    }

    if (throttle_set < 0.010f)
    {
        sum = 0.0f;
    }
    if (ctrl_mode == 0)
    {
        sum = 0.0f;
    }

    error_rate = (error - pre_error) / dt;
    pre_error = error;

    result = angle_pid_mat[2][0] * error + angle_pid_mat[2][1] * sum + angle_pid_mat[2][2] * error_rate;
    return result;
}

float pid_vx(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    if(error < 50 and error > -50){
        sum = sum + error * dt;
    }

    if (sum > 30.0f)
    {
        sum = 30.0;
    }
    if (sum < -30.0f)
    {
        sum = -30.0;
    }

    if (init_waiting > 0)
    {
        sum = 0.0f;
    }

    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;

    if (error_rate > 6.0f)
    {
        error_rate = 6.0f;
    }
    if (error_rate < -6.0f)
    {
        error_rate = -6.0f;
    }

    pre_error = d_error;

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = velocity_pid_mat[1][0] * target + velocity_pid_mat[1][1] * (error + velocity_pid_mat[1][2] * sum +
        velocity_pid_mat[1][3] * d_out);
    return result;
}

float pid_vy(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    if(error < 50 and error > -50){
        sum = sum + error * dt;
    }

    if (sum > 30.0f)
    {
        sum = 30.0;
    }
    if (sum < -30.0f)
    {
        sum = -30.0;
    }

    if (init_waiting > 0)
    {
        sum = 0.0f;
    }

    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;

    if (error_rate > 6.0f)
    {
        error_rate = 6.0f;
    }
    if (error_rate < -6.0f)
    {
        error_rate = -6.0f;
    }

    pre_error = d_error;

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = velocity_pid_mat[1][0] * target + velocity_pid_mat[1][1] * (error + velocity_pid_mat[1][2] * sum +
        velocity_pid_mat[1][3] * d_out);
    return result;
}

float pid_vz(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    if(error < 50 and error > -50){
        sum = sum + error * dt;
    }

    if (sum > 30.0f)
    {
        sum = 30.0;
    }
    if (sum < -30.0f)
    {
        sum = -30.0;
    }
    
    if (init_waiting > 0)
    {
        sum = 0.0f;
    }

    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;

    if (error_rate > 6.0f)
    {
        error_rate = 6.0f;
    }
    if (error_rate < -6.0f)
    {
        error_rate = -6.0f;
    }

    pre_error = d_error;

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = velocity_pid_mat[2][0] * target + velocity_pid_mat[2][1] * (error + velocity_pid_mat[2][2] * sum +
        velocity_pid_mat[2][3] * d_out);
    return result;
}

float pid_pos_x(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    sum = sum + error * dt;

    if (sum > 10.0f)
    {
        sum = 10.0;
    }
    if (sum < -10.0f)
    {
        sum = -10.0;
    }

    if (error > 10.0f)
    {
        sum = 0.0f;
    }
    if (error < -10.0f)
    {
        sum = 0.0f;
    }
    if (init_waiting > 0)
    {
        sum = 0.0f;
    }
    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;
    pre_error = d_error;

    if (error > 6.0f)
    {
        error_rate = 6.0f;
    }
    if (error < -6.0f)
    {
        error_rate = -6.0f;
    }

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = pos_pid_mat[0][0] * target + pos_pid_mat[0][1] * (error + pos_pid_mat[0][2] * sum + pos_pid_mat[0][3] * d_out);

    if (result > 20.0f)
    {
        result = 20.0f;
    }

    if (result < -20.0f)
    {
        result = -20.0f;
    }

    return result;
}

float pid_pos_y(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    sum = sum + error * dt;

    if (sum > 10.0f)
    {
        sum = 10.0;
    }
    if (sum < -10.0f)
    {
        sum = -10.0;
    }

    if (error > 10.0f)
    {
        sum = 0.0f;
    }
    if (error < -10.0f)
    {
        sum = 0.0f;
    }
    if (init_waiting > 0)
    {
        sum = 0.0f;
    }
    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;
    pre_error = d_error;

    if (error > 6.0f)
    {
        error_rate = 6.0f;
    }
    if (error < -6.0f)
    {
        error_rate = -6.0f;
    }

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = pos_pid_mat[1][0] * target + pos_pid_mat[1][1] * (error + pos_pid_mat[1][2] * sum + pos_pid_mat[1][3] * d_out);

    if (result > 20.0f)
    {
        result = 20.0f;
    }

    if (result < -20.0f)
    {
        result = -20.0f;
    }

    return result;
}

float pid_pos_z(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    sum = sum + error * dt;

    if (sum > 5.0f)
    {
        sum = 5.0;
    }
    if (sum < -5.0f)
    {
        sum = -5.0;
    }

    if (error > 10.0f)
    {
        sum = 0.0f;
    }
    if (error < -10.0f)
    {
        sum = 0.0f;
    }
    if (init_waiting > 0)
    {
        sum = 0.0f;
    }
    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;
    pre_error = d_error;

    if (error > 6.0f)
    {
        error_rate = 6.0f;
    }
    if (error < -6.0f)
    {
        error_rate = -6.0f;
    }

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = pos_pid_mat[2][0] * target + pos_pid_mat[2][1] * (error + pos_pid_mat[2][2] * sum + pos_pid_mat[2][3] * d_out);
    
    if(weak_power_state){
        if (result > 4.0f)
        {
            result = 4.0f;
        }

        if (result < -6.0f)
        {
            result = -6.0f;
        }
    }else{
        if (result > 10.0f)
        {
            result = 10.0f;
        }
    
        if (result < -5.0f)
        {
            result = -5.0f;
        }
    }
    return result;
}

float pid_pos_yaw(float target, float real, double dt = 0.01)
{
    static float error;
    static float sum;
    static float pre_error;
    static float result;
    static float error_rate;
    static float d_out_1;
    static float d_out;
    static float d_error;

    error = target - real;

    if (error > PI)
    {
        error = error - 2.0 * PI;
    }
    if (error < -PI)
    {
        error = error + 2.0 * PI;
    }

    sum = sum + error * dt;

    if (sum > 10.0f)
    {
        sum = 10.0;
    }
    if (sum < -10.0f)
    {
        sum = -10.0;
    }

    if (init_waiting > 0)
    {
        sum = 0.0f;
    }

    if (rc_mode != 0)
    {
        if (ctrl_mode != 2)
        {
            sum = 0.0f;
        }
    }

    d_error = 0.0 - real;
    error_rate = (d_error - pre_error) / dt;
    pre_error = d_error;

    if (error_rate > 1.0f)
    {
        error_rate = 1.0f;
    }
    if (error_rate < -1.0f)
    {
        error_rate = -1.0f;
    }

    d_out = pid_N_v * error_rate + (1.0f - pid_N_v) * d_out_1;
    d_out_1 = d_out;

    result = pos_pid_mat[3][0] * target + pos_pid_mat[3][1] * (error + pos_pid_mat[3][2] * sum + pos_pid_mat[3][
        3] * d_out);

    if (result > 10.0f)
    {
        result = 10.0f;
    }
    if (result < -10.0f)
    {
        result = -10.0f;
    }

    return result;
}

float imu_roll;
float imu_pitch;
float imu_yaw;
float target_velocity_roll = 0.0f;
float target_velocity_pitch = 0.0f;
float target_velocity_yaw = 0.0f;

float target_angle_roll = 0.0f;
float target_angle_pitch = 0.0f;
float target_w_yaw = 0.0f;

Quaternion yaw_to_quaternion(double yaw)
{
    Quaternion quaternion;
    quaternion.w = cos(yaw / 2);
    quaternion.x = 0;
    quaternion.y = 0;
    quaternion.z = sin(yaw / 2);
    return quaternion;
}

// Function to convert pitch angle (rotation around Y-axis) to quaternion
Quaternion pitch_to_quaternion(double pitch)
{
    Quaternion quaternion;
    quaternion.w = cos(pitch / 2);
    quaternion.x = 0;
    quaternion.y = sin(pitch / 2);
    quaternion.z = 0;
    return quaternion;
}

Quaternion roll_to_quaternion(double roll)
{
    Quaternion quaternion;
    quaternion.w = cos(roll / 2);
    quaternion.x = sin(roll / 2);
    quaternion.y = 0;
    quaternion.z = 0;
    return quaternion;
}

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

Quaternion quaternion_diff(Quaternion q1, Quaternion q2)
{
    q1 = quaternion_conjugate(q1);
    Quaternion result = multiply_quaternion(&q2, &q1);
    if (result.w < 0.0f)
    {
        result.w = -result.w;
        result.x = -result.x;
        result.y = -result.y;
        result.z = -result.z;
    }
    return result;
}

void quaternionToAngles(Quaternion q, float* roll, float* pitch, float* yaw)
{
    float we = q.w;
    if (we > 0.999999f)
    {
        we = 0.999999f;
    }
    if (we < -0.999999f)
    {
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

void World_to_Body(float* vector_e, float* vector_v, Quaternion Qin)
{
    float C11, C12, C13;
    float C21, C22, C23;
    float C31, C32, C33;

    float Q[4];

    Q[0] = Qin.w;
    Q[1] = -Qin.x;
    Q[2] = -Qin.y;
    Q[3] = -Qin.z;

    C11 = Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
    C12 = 2.0f * (Q[1] * Q[2] - Q[0] * Q[3]);
    C13 = 2.0f * (Q[1] * Q[3] + Q[0] * Q[2]);

    C21 = 2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
    C22 = Q[0] * Q[0] - Q[1] * Q[1] + Q[2] * Q[2] - Q[3] * Q[3];
    C23 = 2.0f * (Q[2] * Q[3] - Q[0] * Q[1]);

    C31 = 2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
    C32 = 2.0f * (Q[2] * Q[3] + Q[0] * Q[1]);
    C33 = Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];

    vector_v[0] = C11 * vector_e[0] + C12 * vector_e[1] + C13 * vector_e[2];
    vector_v[1] = C21 * vector_e[0] + C22 * vector_e[1] + C23 * vector_e[2];
    vector_v[2] = C31 * vector_e[0] + C32 * vector_e[1] + C33 * vector_e[2];
}

quadrotor_msgs::GoalSet ego_goal_msg;

int mission_step = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "basic_control");
    ros::NodeHandle node_handle;
    rc_mode = 0;
    ctrl_mode = 2;
    pid_init();
    BasicControl control(&node_handle);
    return 0;
}

BasicControl::BasicControl(ros::NodeHandle* nh)
{
    pwm_cmd.rotorPWM0 = 0.1;
    pwm_cmd.rotorPWM1 = 0.1;
    pwm_cmd.rotorPWM2 = 0.1;
    pwm_cmd.rotorPWM3 = 0.1;

    rc_channel1_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc1", 1,
                                                         std::bind(&BasicControl::channel1_callback, this,
                                                                   std::placeholders::_1));
    rc_channel2_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc2", 1,
                                                         std::bind(&BasicControl::channel2_callback, this,
                                                                   std::placeholders::_1));
    rc_channel3_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc3", 1,
                                                         std::bind(&BasicControl::channel3_callback, this,
                                                                   std::placeholders::_1));
    rc_channel4_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc4", 1,
                                                         std::bind(&BasicControl::channel4_callback, this,
                                                                   std::placeholders::_1));
    rc_channel5_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc5", 1,
                                                         std::bind(&BasicControl::channel5_callback, this,
                                                                   std::placeholders::_1));
    rc_channel6_suber = nh->subscribe<std_msgs::Float32>("/custom_debug/rc6", 1,
                                                         std::bind(&BasicControl::channel6_callback, this,
                                                                   std::placeholders::_1));
    pose_suber = nh->subscribe<nav_msgs::Odometry>("/odometry/filtered", 1,
                                                   std::bind(&BasicControl::poseCallback, this, std::placeholders::_1));
    // TODO(change to estimated pose)
    nh->setParam("/custom_debug/rc_mode", -1); // TODO(change to 0 later)
    imu_suber = nh->subscribe<sensor_msgs::Imu>("/airsim_node/drone_1/imu/imu", 1,
                                                std::bind(&BasicControl::imuCallback, this, std::placeholders::_1));
    pwm_publisher = nh->advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 1);

    tf_cmd_suber = nh->subscribe<std_msgs::Float32MultiArray>("/exe/cmd", 1,
                                                              std::bind(&BasicControl::tf_cmd_callback, this,
                                                                        std::placeholders::_1));

    no_g_acc_suber = nh->subscribe<std_msgs::Float32MultiArray>("/exe/output_acc", 1,
                                                              std::bind(&BasicControl::no_g_acc_callback, this,
                                                                        std::placeholders::_1));

    start_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1,
                                                              std::bind(&BasicControl::start_pose_callback, this,
                                                                        std::placeholders::_1));

    end_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1,
                                                              std::bind(&BasicControl::end_pose_callback, this,
                                                                        std::placeholders::_1));

    rate_x_target_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/target_rate_x", 10);
    rate_y_target_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/target_rate_y", 10);
    rate_z_target_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/target_rate_z", 10);
    rate_x_real_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/real_rate_x", 10);
    rate_y_real_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/real_rate_y", 10);
    rate_z_real_publisher = nh->advertise<std_msgs::Float32>("/custom_debug/real_rate_z", 10);
    exe_path_publisher = nh->advertise<nav_msgs::Path>("/exe/path", 10);
    pcl_enbale_publisher = nh->advertise<std_msgs::Bool>("/pcl/enable", 10);

    planner_acc_limit_publisher = nh->advertise<std_msgs::Float32>("/exe/acc_limit", 10);
    planner_vel_limit_publisher = nh->advertise<std_msgs::Float32>("/exe/vel_limit", 10);

    world_force_marker_publisher = nh->advertise<visualization_msgs::Marker>("/exe/force_marker", 10);

    initButterworthFilter(&world_vel_x_filter, 100.0, 1.25);
    initButterworthFilter(&world_vel_y_filter, 100.0, 1.25);
    initButterworthFilter(&world_vel_z_filter, 100.0, 1.25);

    initButterworthFilter(&world_pos_x_filter, 100.0, 1.25);
    initButterworthFilter(&world_pos_y_filter, 100.0, 1.25);
    initButterworthFilter(&world_pos_z_filter, 100.0, 1.25);

    rc_mode_timer = nh->createTimer(ros::Duration(0.05),
                                    std::bind(&BasicControl::rc_mode_check_callback, this, std::placeholders::_1));
    pwm_send_timer = nh->createTimer(ros::Duration(0.0125),
                                     std::bind(&BasicControl::pwm_send_callback, this, std::placeholders::_1));

    scheduler_timer = nh->createTimer(ros::Duration(0.1), 
                                     std::bind(&BasicControl::scheduler_callback, this, std::placeholders::_1));

    for (int i = 0; i < 11; i++)
    {
        tf_cmd[i] = NAN;
    }

    tf2_ros::TransformListener tfListener(tfBuffer);

    path01_init();
    path02_init();
    path03_init();
    path04_init();
    path05_init();
    path06_init();
    path07_init();
    path08_init();
    path09_init();
    path10_init();
    path11_init();
    path12_init();


    ros::spin();
}

BasicControl::~BasicControl()
{
}

float target_vel_x;
float target_vel_y;
float target_vel_z;

float hover_throttle = 0.18;

float mag_vector(float input_vec[])
{
    return sqrt(input_vec[0] * input_vec[0] + input_vec[1] * input_vec[1] + input_vec[2] * input_vec[2]);
}

mission_point get_point_by_B_and_u(mission_point B[], double u) {
	if (u < 0.0) {
		u = 0.0;
	}
	if (u > 1.0) {
		u = 1.0;
	}
	double b03 = pow(1.0 - u, 3);
	double b13 = 3.0 * u * pow(1.0 - u, 2);
	double b23 = 3.0 * u * u * (1.0 - u);
	double b33 = u * u * u;
	mission_point result;
	result.position[0] = b03 * B[0].position[0] + b13 * B[1].position[0] + b23 * B[2].position[0] + b33 * B[3].position[0];
	result.position[1] = b03 * B[0].position[1] + b13 * B[1].position[1] + b23 * B[2].position[1] + b33 * B[3].position[1];
	result.position[2] = b03 * B[0].position[2] + b13 * B[1].position[2] + b23 * B[2].position[2] + b33 * B[3].position[2];
	return result;
}

void normalize_vec(float input_vec[], float output_vec[])
{
    float magnitude = mag_vector(input_vec);
    output_vec[0] /= magnitude;
    output_vec[1] /= magnitude;
    output_vec[2] /= magnitude;
}

void cross_product(float input_vec1[], float input_vec2[], float output_vec[])
{
    output_vec[0] = input_vec1[1] * input_vec2[2] - input_vec1[2] * input_vec2[1];
    output_vec[1] = input_vec1[2] * input_vec2[0] - input_vec1[0] * input_vec2[2];
    output_vec[2] - input_vec1[0] * input_vec2[1] - input_vec1[1] * input_vec2[0];
}

float dot_product(float input_vec1[], float input_vec2[])
{
    return input_vec1[0] * input_vec2[0] + input_vec1[1] * input_vec2[1] + input_vec1[2] * input_vec2[2];
}

Quaternion vector_to_quaternion(float input_vec[], float angle)
{
    Quaternion quaternion;
    float half_angle = angle / 2.0;
    float sin_half_angle = sin(half_angle);
    quaternion.w = cos(half_angle);
    quaternion.x = input_vec[0] * sin_half_angle;
    quaternion.y = input_vec[1] * sin_half_angle;
    quaternion.z = input_vec[2] * sin_half_angle;
    return quaternion;
}

void BasicControl::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    static double pre_time;
    double imu_time = msg->header.stamp.toSec();
    double dt = imu_time - pre_time;

    if(dt < 0.003){
        dt = 0.003;
    }
    if(dt > 0.03){
        dt = 0.03;
    }

    pre_time = imu_time;
    float target_vel_body[3];
    if (ctrl_mode == 2 and rc_mode == 1)
    {
        target_vel_body[1] = rc_channel[0] / -40.0f;
        target_vel_body[0] = rc_channel[1] / 40.0f;
        target_w_yaw = rc_channel[3] * -0.002341f;
        target_vel_body[2] = rc_channel[2] / 80.0f;
        if (target_vel_body[2] < 0.2 and target_vel_body[2] > -0.2)
        {
            target_vel_body[2] = 0.0;
        }
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

    float measure_yaw = 0.0f;
    measure_yaw = atan2(2.0 * (odom_pose.w * odom_pose.z + odom_pose.x * odom_pose.y),
                        1.0 - 2.0 * (odom_pose.y * odom_pose.y + odom_pose.z * odom_pose.z));

    measure_world_pos[0] = applyButterworthFilter(&world_pos_x_filter, msg->pose.pose.position.x);
    measure_world_pos[1] = applyButterworthFilter(&world_pos_y_filter, msg->pose.pose.position.y);
    measure_world_pos[2] = applyButterworthFilter(&world_pos_z_filter, msg->pose.pose.position.z);
    measure_world_pos[3] = measure_yaw;

    Quaternion yaw_quaternion = yaw_to_quaternion(measure_yaw);
    Quaternion body_to_yaw = yaw_to_quaternion(-measure_yaw);
    Quaternion de_yaw_ahrs = multiply_quaternion(&body_to_yaw, &odom_pose);

    Quaternion body_to_world = quaternion_conjugate(odom_pose);
    float world_measure_vel[3];
    World_to_Body(body_measure_vel, world_measure_vel, body_to_world);

    double filtered_vel[3];
    filtered_vel[0] = applyButterworthFilter(&world_vel_x_filter, world_measure_vel[0]);
    filtered_vel[1] = applyButterworthFilter(&world_vel_y_filter, world_measure_vel[1]);
    filtered_vel[2] = applyButterworthFilter(&world_vel_z_filter, world_measure_vel[2]);


    world_measure_vel[0] = filtered_vel[0];
    world_measure_vel[1] = filtered_vel[1];
    world_measure_vel[2] = filtered_vel[2];

    float world_target_vel[3];
    World_to_Body(target_vel_body, world_target_vel, body_to_yaw);

    float world_force[3];
    float body_force[3];

    if (rc_mode == -1)
    {
        if (!isnan(tf_cmd[0]))
        {
            if (!isnan(tf_cmd[1]))
            {
                if (!isnan(tf_cmd[2]))
                {
                    world_target_vel[0] = pid_pos_x(tf_cmd[0], measure_world_pos[0], dt);
                    world_target_vel[1] = pid_pos_y(tf_cmd[1], measure_world_pos[1], dt);
                    world_target_vel[2] = pid_pos_z(tf_cmd[2], measure_world_pos[2], dt);
                }
            }
        }
    }

    if (rc_mode == -1)
    {
        if (!isnan(tf_cmd[3]))
        {
            if (!isnan(tf_cmd[4]))
            {
                if (!isnan(tf_cmd[5]))
                {
                    world_target_vel[0] += tf_cmd[3];
                    world_target_vel[1] += tf_cmd[4];
                    world_target_vel[2] += tf_cmd[5];
                }
            }
        }
    }

    world_force[0] = pid_vx(world_target_vel[0], world_measure_vel[0], dt);
    world_force[1] = pid_vy(world_target_vel[1], world_measure_vel[1], dt);
    world_force[2] = pid_vz(world_target_vel[2], world_measure_vel[2], dt);



    std_msgs::Float32 rate_msg;

    rate_msg.data = world_measure_vel[0];
    rate_x_real_publisher.publish(rate_msg);
    rate_msg.data = world_target_vel[0];
    rate_x_target_publisher.publish(rate_msg);
    rate_msg.data = world_measure_vel[1];
    rate_y_real_publisher.publish(rate_msg);
    rate_msg.data = world_target_vel[1];
    rate_y_target_publisher.publish(rate_msg);
    rate_msg.data = world_measure_vel[2];
    rate_z_real_publisher.publish(rate_msg);
    rate_msg.data = world_target_vel[2];
    rate_z_target_publisher.publish(rate_msg);

    if (rc_mode == -1)
    {
        if (!isnan(tf_cmd[6]))
        {
            if (!isnan(tf_cmd[7]))
            {
                if (!isnan(tf_cmd[8]))
                {
                    if(weak_power_state){
                        world_force[0] += tf_cmd[6] * 0.8 / 12.335; //12:all max thrust
                        world_force[1] += tf_cmd[7] * 0.8 / 12.335; //0.8:mass 12.335:waek mode max force
                        world_force[2] += tf_cmd[8] * 0.8 / 12.335; //output throttle 0.0->1.0
                    }
                    else{
                        world_force[0] += tf_cmd[6] * 0.8 / 49.0; //49:all max thrust, strong power mode
                        world_force[1] += tf_cmd[7] * 0.8 / 49.0;
                        world_force[2] += tf_cmd[8] * 0.8 / 49.0;
                    }                    
                }
            }
        }
    }

    float body_anti_drag_force[3];
    float world_anti_drag_force[3];
    if(weak_power_state){
        body_anti_drag_force[0] = 7.2e-3 * body_measure_vel[0] * body_measure_vel[0] / 12.335;
        body_anti_drag_force[1] = 7.2e-3 * body_measure_vel[1] * body_measure_vel[1] / 12.335;
        body_anti_drag_force[2] = 0.109 * body_measure_vel[2] * body_measure_vel[2] / 12.335;
    }
    else{
        body_anti_drag_force[0] = 7.2e-3 * body_measure_vel[0] * body_measure_vel[0] / 49.0;
        body_anti_drag_force[1] = 7.2e-3 * body_measure_vel[1] * body_measure_vel[1] / 49.0;
        body_anti_drag_force[2] = 0.109 * body_measure_vel[2] * body_measure_vel[2] / 49.0;
    }
    World_to_Body(body_anti_drag_force, world_anti_drag_force, body_to_world);

    world_force[0] += world_anti_drag_force[0];
    world_force[1] += world_anti_drag_force[1];
    world_force[2] += world_anti_drag_force[2];

    if (rc_mode == -1)
    {
        if (!isnan(tf_cmd[9]))
        {
            target_w_yaw = pid_pos_yaw(tf_cmd[9], measure_world_pos[3], dt);
        }
        else
        {
            target_w_yaw = 0.0;
        }
        if (target_w_yaw > 1.6)
        {
            target_w_yaw = 1.6;
        }

        if (target_w_yaw < -1.6)
        {
            target_w_yaw = -1.6;
        }
    }

    world_force[2] = world_force[2] + hover_throttle;
    if (world_force[2] < 0.01f)
    {
        world_force[2] = 0.01f;
    }
    if (world_force[2] > 0.99f)
    {
        world_force[2] = 0.99f;
    }

    if (world_force[0] * world_force[0] + world_force[1] * world_force[1] > 1.0 - world_force[2] * world_force[2])
    {
        float factor = (1.0 - world_force[2] * world_force[2]) / (world_force[0] * world_force[0] + world_force[1] *
            world_force[1]);
        world_force[0] *= factor;
        world_force[1] *= factor;
    }

    visualization_msgs::Marker world_force_marker;
    world_force_marker.header.stamp = ros::Time::now();
    world_force_marker.header.frame_id = "base_link_identity";
    world_force_marker.id = 0;
    world_force_marker.ns = "vector";
    world_force_marker.type = visualization_msgs::Marker::ARROW;
    world_force_marker.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Point start_point, end_point;
    start_point.x = 0.0;
    start_point.y = 0.0;
    start_point.z = 0.0;

    end_point.x = 3.0 * world_force[0];
    end_point.y = 3.0 * world_force[1];
    end_point.z = 3.0 * world_force[2];

    world_force_marker.points.push_back(start_point);
    world_force_marker.points.push_back(end_point);

    world_force_marker.color.r = 1.0;
    world_force_marker.color.g = 0.0;
    world_force_marker.color.b = 0.0;
    world_force_marker.color.a = 1.0;
    world_force_marker.scale.x = 0.1;
    world_force_marker.scale.y = 0.1;
    world_force_marker.scale.z = 2.0;

    world_force_marker_publisher.publish(world_force_marker);

    World_to_Body(world_force, body_force, yaw_quaternion);
    throttle_set = sqrt(body_force[0] * body_force[0] + body_force[1] * body_force[1] + body_force[2] * body_force[2]);

    float norm_body_force[3];
    norm_body_force[0] = body_force[0] / throttle_set;
    norm_body_force[1] = body_force[1] / throttle_set;
    norm_body_force[2] = body_force[2] / throttle_set;

    float theta = acos(norm_body_force[2]);
    float sin_half_theta = sin(theta / 2.0f);
    float cos_half_theta = cos(theta / 2.0f);
    float norm_xy = sqrt(body_force[0] * body_force[0] + body_force[1] * body_force[1]);

    target_quaternion.w = cos_half_theta;
    target_quaternion.y = -sin_half_theta * body_force[0] / norm_xy;
    target_quaternion.x = -sin_half_theta * body_force[1] / norm_xy;
    target_quaternion.z = 0.0f;

    float up_vector[3];
    float norm_tilt_vector[3];
    up_vector[0] = 0.0;
    up_vector[1] = 0.0;
    up_vector[2] = 1.0;
    World_to_Body(up_vector, norm_tilt_vector, de_yaw_ahrs);
    float cos_error = norm_tilt_vector[0] * norm_body_force[0] + norm_tilt_vector[1] * norm_body_force[1] +
        norm_tilt_vector[2] * norm_body_force[2];

    // throttle_set = throttle_set * pow(cos_error, 0.25);// * throttle_set);

    if (throttle_set > 1.0)
    {
        throttle_set = 1.0;
    }
}

void BasicControl::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    static double pre_time;
    double imu_time = msg->header.stamp.toSec();
    double dt = imu_time - pre_time;

    if(dt < 0.003){
        dt = 0.003;
    }
    if(dt > 0.03){
        dt = 0.03;
    }

    pre_time = imu_time;
    measure_quaternion.w = msg->orientation.w;
    measure_quaternion.x = msg->orientation.x;
    measure_quaternion.y = msg->orientation.y;
    measure_quaternion.z = msg->orientation.z;

    no_g_acc[0] = msg->linear_acceleration.x;
    no_g_acc[1] = msg->linear_acceleration.y;
    no_g_acc[2] = msg->linear_acceleration.z;

    float measure_yaw = 0.0f;
    measure_yaw = atan2(
        2.0 * (measure_quaternion.w * measure_quaternion.z + measure_quaternion.x * measure_quaternion.y),
        1.0 - 2.0 * (measure_quaternion.y * measure_quaternion.y + measure_quaternion.z * measure_quaternion.z));
    imu_angle[2] = measure_yaw;
    imu_angle[0] = atan2(2.0 * (measure_quaternion.w * measure_quaternion.x + measure_quaternion.y * measure_quaternion.z),
        1.0 - 2.0 * (measure_quaternion.x * measure_quaternion.x + measure_quaternion.y * measure_quaternion.y));
    imu_angle[1] = asin(2.0 * (measure_quaternion.w * measure_quaternion.y - measure_quaternion.z * measure_quaternion.x));
    Quaternion de_yaw_quaternion = yaw_to_quaternion(-measure_yaw);
    Quaternion de_yaw_ahrs = multiply_quaternion(&de_yaw_quaternion, &measure_quaternion);

    if (ctrl_mode == 1 and rc_mode == 1)
    {
        target_angle_roll = rc_channel[0] * 1.5e-3;
        target_angle_pitch = rc_channel[1] * -1.5e-3;
        target_w_yaw = rc_channel[3] * -0.002341f;
        if (rc_channel[4] > 100)
        {
            throttle_set = (rc_channel[2] / 2.0 + 500.0) / 1000.0;
        }
        else
        {
            throttle_set = (rc_channel[2] / 2.0 + 500.0) / 2000.0;
        }

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

    Quaternion diff_quaternion = quaternion_diff(de_yaw_ahrs, target_quaternion);
    quaternionToAngles(diff_quaternion, &error_angle[0], &error_angle[1], &error_angle[2]);
    World_to_Body(error_angle, error_body, de_yaw_ahrs);

    if (isnan(error_body[0]))
    {
        error_body[0] = 0.0f;
    }
    if (isnan(error_body[1]))
    {
        error_body[1] = 0.0f;
    }
    if (isnan(error_body[2]))
    {
        error_body[2] = 0.0f;
    }

    w_yaw_world[0] = 0.0f;
    w_yaw_world[1] = 0.0f;
    w_yaw_world[2] = target_w_yaw;
    World_to_Body(w_yaw_world, w_yaw_body, measure_quaternion);

    if (isnan(w_yaw_body[0]))
    {
        w_yaw_body[0] = 0.0f;
    }
    if (isnan(w_yaw_body[1]))
    {
        w_yaw_body[1] = 0.0f;
    }
    if (isnan(w_yaw_body[2]))
    {
        w_yaw_body[2] = 0.0f;
    }

    target_velocity_roll = pid_angle_roll(error_body[0], dt) + w_yaw_body[0];
    target_velocity_pitch = -pid_angle_pitch(error_body[1], dt) - w_yaw_body[1];
    target_velocity_yaw = pid_angle_yaw(error_body[2], dt) + w_yaw_body[2];

    if(target_velocity_roll > 3.8){
        target_velocity_roll = 3.8;
    }
    if(target_velocity_roll < -3.8){
        target_velocity_roll = -3.8;
    }
    if(target_velocity_pitch > 3.8){
        target_velocity_pitch = 3.8;
    }
    if(target_velocity_pitch < -3.8){
        target_velocity_pitch = -3.8;
    }
    if(target_velocity_yaw > 3.8){
        target_velocity_yaw = 3.8;
    }
    if(target_velocity_yaw < -3.8){
        target_velocity_yaw = -3.8;
    }

    gyro_data[0] = msg->angular_velocity.x;
    gyro_data[1] = msg->angular_velocity.y;
    gyro_data[2] = msg->angular_velocity.z;

    if (ctrl_mode == 0 and rc_mode == 1)
    {
        target_velocity_roll = rc_channel[0] * 0.004;
        target_velocity_pitch = rc_channel[1] * 0.004;
        target_velocity_yaw = rc_channel[3] * -0.004;
        if (rc_channel[4] > 100)
        {
            throttle_set = (rc_channel[2] / 2.0 + 500.0) / 1000.0;
        }
        else
        {
            throttle_set = (rc_channel[2] / 2.0 + 500.0) / 2000.0;
        }
    }

    imu_roll = gyro_data[0];
    imu_pitch = -gyro_data[1];
    imu_yaw = -gyro_data[2];
    float roll_in = kalman_roll(imu_roll);
    float pitch_in = kalman_pitch(imu_pitch);
    float yaw_in = kalman_yaw(imu_yaw);

    //    std::cout << "roll_in: " << roll_in << std::endl;
    float output_roll = pid_roll(target_velocity_roll, roll_in, dt) / 500.0;
    float output_pitch = pid_pitch(target_velocity_pitch, pitch_in, dt) / 500.0;
    float output_yaw = pid_yaw(target_velocity_yaw, yaw_in, dt) / 500.0;

    float front_left_speed = 0.0 + output_roll - output_pitch + output_yaw + throttle_set;
    float front_right_speed = 0.0 - output_roll - output_pitch - output_yaw + throttle_set;
    float rear_left_speed = 0.0 + output_roll + output_pitch - output_yaw + throttle_set;
    float rear_right_speed = 0.0 - output_roll + output_pitch + output_yaw + throttle_set;

    if (front_left_speed < motor_idle_speed)
    {
        front_left_speed = motor_idle_speed;
    }
    if (front_right_speed < motor_idle_speed)
    {
        front_right_speed = motor_idle_speed;
    }
    if (rear_left_speed < motor_idle_speed)
    {
        rear_left_speed = motor_idle_speed;
    }
    if (rear_right_speed < motor_idle_speed)
    {
        rear_right_speed = motor_idle_speed;
    }
    if (front_left_speed > motor_max_speed)
    {
        front_left_speed = motor_max_speed;
    }
    if (front_right_speed > motor_max_speed)
    {
        front_right_speed = motor_max_speed;
    }
    if (rear_left_speed > motor_max_speed)
    {
        rear_left_speed = motor_max_speed;
    }
    if (rear_right_speed > motor_max_speed)
    {
        rear_right_speed = motor_max_speed;
    }

    pwm_cmd.rotorPWM0 = front_right_speed;
    pwm_cmd.rotorPWM1 = rear_left_speed;
    pwm_cmd.rotorPWM2 = front_left_speed;
    pwm_cmd.rotorPWM3 = rear_right_speed;
}

void BasicControl::pwm_send_callback(const ros::TimerEvent& event)
{
    if (init_waiting > 0)
    {
        ROS_INFO("UKF Waiting...");
    }
    else
    {
        if (rc_mode != 0)
        {
            pwm_publisher.publish(pwm_cmd);
        }
    }
}

void BasicControl::channel1_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[0] = msg->data;
}

void BasicControl::channel2_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[1] = msg->data;
}

void BasicControl::channel3_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[2] = msg->data;
}

void BasicControl::channel4_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[3] = msg->data;
}

void BasicControl::channel5_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[4] = msg->data;
}

void BasicControl::channel6_callback(const std_msgs::Float32::ConstPtr& msg)
{
    rc_channel[5] = msg->data;
}

void BasicControl::tf_cmd_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (rc_mode == -1)
    {
        for (int i = 0; i < 11; i++)
        {
            tf_cmd[i] = msg->data[i];
        }
    }
}

void BasicControl::no_g_acc_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    no_g_acc[0] = msg->data[0];
    no_g_acc[1] = msg->data[1];
    no_g_acc[2] = msg->data[2];

    float total_u = pwm_cmd.rotorPWM0 + pwm_cmd.rotorPWM1 + pwm_cmd.rotorPWM2 + pwm_cmd.rotorPWM3;
    float k = total_u / abs(no_g_acc[2]); //sqrt(no_g_acc[0] * no_g_acc[0] + no_g_acc[1] * no_g_acc[1] + no_g_acc[2] * no_g_acc[2]);

    k = k * cos(imu_angle[0]) * cos(imu_angle[1]);

    static bool pre_state;

    // std_msgs::Float32 rate_msg;
    // rate_msg.data = k;
    // rate_z_target_publisher.publish(rate_msg);

    if(weak_power_state){
        if(total_u < 2.86){//min than 2.86
            weak_power_state = (k>0.27);
        }else{
            weak_power_state = (k>0.25);
        }
    }else{
        if(total_u > 0.72){//max than 0.72
            weak_power_state = (k>0.10);
        }else{
            weak_power_state = (k>0.35);
        }
    }

    if(force_strong_power_mode){
        weak_power_state = false;
    }
    if(force_weak_power_mode){
        weak_power_state = true;
    }

    //std::cout<<"a:"<<no_g_acc[2]<<" u:"<<total_u<<", k:"<< k <<",weak state:"<<weak_power_state<<std::endl;

    if(weak_power_state != pre_state){
        if(weak_power_state){
            pid_weak();
            hover_throttle = 0.715;
        }else{
            pid_init();
            hover_throttle = 0.18;
        }
    }

    pre_state = weak_power_state;
}

void BasicControl::rviz_clicked_point_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ego_goal_msg.drone_id = 0;
    ego_goal_msg.goal[0] = msg->pose.position.x;
    ego_goal_msg.goal[1] = msg->pose.position.y;
    ego_goal_msg.goal[2] = msg->pose.position.z;
    quad_goal_publisher.publish(ego_goal_msg);
}

void BasicControl::rc_mode_check_callback(const ros::TimerEvent& event)
{
    if (init_waiting > 0)
    {
        init_waiting--;
    }
    int got_mode;
    ros::NodeHandle nh;
    nh.getParam("/custom_debug/rc_mode", got_mode);

    if (got_mode == -1)
    {
        ctrl_mode = -1;
        rc_mode = -1;
    }

    if (got_mode == 0)
    {
        ctrl_mode = 2;
        rc_mode = 0;
    }

    if (got_mode > 0)
    {
        rc_mode = 1;
        switch (got_mode)
        {
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
    if (rc_mode != -1)
    {
        tf_cmd[0] = measure_world_pos[0];
        tf_cmd[1] = measure_world_pos[1];
        tf_cmd[2] = measure_world_pos[2];
        tf_cmd[3] = NAN;
        tf_cmd[4] = NAN;
        tf_cmd[5] = NAN;
        tf_cmd[6] = NAN;
        tf_cmd[7] = NAN;
        tf_cmd[8] = NAN;
        tf_cmd[9] = measure_world_pos[3];
        tf_cmd[10] = NAN;
    }
    // std::cout<<got_mode<<std::endl;
    // std::cout<<ctrl_mode<<" "<<rc_mode <<std::endl;
}

void BasicControl::start_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    start_pose[0] = msg->pose.position.y;
    start_pose[1] = msg->pose.position.x;
    start_pose[2] = - msg->pose.position.z;
}

void BasicControl::end_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // end_pose[0] = msg->pose.position.y;
    // end_pose[1] = msg->pose.position.x;
    // end_pose[2] = - msg->pose.position.z;
}

int mission_cnt = 0;

auto exe_path1 = mission_path_list.begin();//from start
auto exe_path2 = mission_path_list.begin();//from end
nav_msgs::Path mission_path_1;
nav_msgs::Path mission_path_2;
nav_msgs::Path mission_path_3;
nav_msgs::Path mission_path_4;

int start_id = 0;
int end_id = 0;

void BasicControl::scheduler_callback(const ros::TimerEvent& event)//4HZ 0.2s
{
    std::cout<<"mission step:"<<mission_step<<std::endl;
    if (mission_step == 0)
    {
        rc_mode = -1;
        mission_step = 1001;
        return;
    }
    if (rc_mode ==-1)
    {
        if (mission_step == 1001)
        {
            if (point_distance(start_pose, measure_world_pos) < 10.0)
            {
                mission_cnt = mission_cnt+1;
                if (mission_cnt > 50 or init_waiting <= 0)
                {
                    mission_cnt = 0;
                    init_waiting = 0;

                    geometry_msgs::TransformStamped base2plane;
                    try
                    {
                        base2plane = tfBuffer.lookupTransform("map", "start_by", ros::Time(0));
                    }
                    catch (tf2::TransformException& ex)
                    {
                        ROS_WARN("Control Get TF ERROR!");
                        return;
                    }
                    start_by_pose[0] = base2plane.transform.translation.x;
                    start_by_pose[1] = base2plane.transform.translation.y;
                    start_by_pose[2] = base2plane.transform.translation.z;
                    try
                    {
                        base2plane = tfBuffer.lookupTransform("map", "end_by", ros::Time(0));
                    }
                    catch (tf2::TransformException& ex)
                    {
                        ROS_WARN("Control Get TF ERROR!");
                        return;
                    }
                    end_by_pose[0] = base2plane.transform.translation.x;
                    end_by_pose[1] = base2plane.transform.translation.y;
                    end_by_pose[2] = base2plane.transform.translation.z;
                    try
                    {
                        base2plane = tfBuffer.lookupTransform("map", "real_end", ros::Time(0));
                    }
                    catch (tf2::TransformException& ex)
                    {
                        ROS_WARN("Control Get TF ERROR!");
                        return;
                    }
                    end_pose[0] = base2plane.transform.translation.x;
                    end_pose[1] = base2plane.transform.translation.y;
                    end_pose[2] = base2plane.transform.translation.z;

                    target_world_pos[0] = start_by_pose[0];
                    target_world_pos[1] = start_by_pose[1];
                    target_world_pos[2] = start_by_pose[2] + 1.0;

                    nav_msgs::Path direct_path;
                    direct_path.header.stamp = ros::Time::now();
                    direct_path.header.frame_id = "map";

                    geometry_msgs::PoseStamped path_point;
                    path_point.pose.position.x = start_pose[0];
                    path_point.pose.position.y = start_pose[1];
                    path_point.pose.position.z = start_pose[2] + 2.5;
                    path_point.pose.orientation.w = 1.0;
                    direct_path.poses.push_back(path_point);

                    path_point.pose.position.x = target_world_pos[0];
                    path_point.pose.position.y = target_world_pos[1];
                    path_point.pose.position.z = target_world_pos[2];
                    path_point.pose.orientation.w = 1.0;
                    direct_path.poses.push_back(path_point);

                    exe_path_publisher.publish(direct_path);
                    mission_step = 10021;
                }
            }
            return;
        }
        if (mission_step == 10021)
        {
            static int take_off_cnt = 0;
            if (!isnan(tf_cmd[5]))
            {
                take_off_cnt = take_off_cnt + 1;
                if (take_off_cnt > 5)
                {
                    std_msgs::Bool enable_flag;
                    enable_flag.data =  true;
                    pcl_enbale_publisher.publish(enable_flag);
                }
            }
            if (point_distance(target_world_pos, measure_world_pos) < 3.0)
            {
                target_world_pos[0] = start_by_pose[0];
                target_world_pos[1] = start_by_pose[1];
                target_world_pos[2] = start_by_pose[2] + 5.0;

                nav_msgs::Path direct_path;
                direct_path.header.stamp = ros::Time::now();
                direct_path.header.frame_id = "map";

                geometry_msgs::PoseStamped path_point;
                path_point.pose.position.x = start_by_pose[0];
                path_point.pose.position.y = start_by_pose[1];
                path_point.pose.position.z = start_by_pose[2] + 1.0;
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                path_point.pose.position.x = target_world_pos[0];
                path_point.pose.position.y = target_world_pos[1];
                path_point.pose.position.z = target_world_pos[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                exe_path_publisher.publish(direct_path);
                mission_step = 1002;
            }
            return;
        }
        if (mission_step == 1002)
        {

            if (point_distance(target_world_pos, measure_world_pos) < 2.0)
            {
                std::cout<<"start pose "<<start_pose[0]<<", "<<start_pose[1]<<", "<<start_pose[2]<<std::endl;
                std::cout<<"end pose "<<end_pose[0]<<", "<<end_pose[1]<<", "<<end_pose[2]<<std::endl;
                float min_distance = 5000.0;
                int i = 0;
                int id = 0;

                for (auto path = mission_path_list.begin(); path != mission_path_list.end(); ++path)
                {
                    i = i + 1;
                    float path_begin_point[] = {path->front().position[0], path->front().position[1], path->front().position[2]};
                    std::cout<<std::endl<<"min distance"<<min_distance<<std::endl;
                    std::cout<<"path"<<i<<" pose "<< path_begin_point[0]<<", "<< path_begin_point[1]<<", "<< path_begin_point[2] <<std::endl;
                    float current_distance = point_distance(start_pose, path_begin_point);
                    std::cout<<"current distance: "<< current_distance << std::endl;

                    if (current_distance < min_distance)
                    {
                        exe_path1 = path;
                        min_distance = current_distance;
                        id = i;
                    }
                    std::cout<<"min distance"<<min_distance<<std::endl;

                }
                std::cout<<"go path id: "<<id<<std::endl;
                start_id = id -1;
                min_distance = 99999.0;
                i = 0;
                for (auto path = mission_path_list.begin(); path != mission_path_list.end(); path++)
                {
                    i = i + 1;
                    std::cout<<std::endl<<"min distance"<<min_distance<<std::endl;
                    float path_begin_point[] = {path->front().position[0], path->front().position[1], path->front().position[2]};
                    float current_distance = point_distance(end_pose, path_begin_point);
                    if (current_distance < min_distance)
                    {
                        exe_path2 = path;
                        min_distance = current_distance;
                        id = i;
                    }
                }
                std::cout<<"back path id: "<<id<<std::endl;
                end_id = id -1;

                mission_path_1.header.stamp = ros::Time::now();
                mission_path_1.header.frame_id = "map";

                for (auto point = exe_path1->begin(); point != exe_path1->end(); ++point)
                {
                    geometry_msgs::PoseStamped path_point;
                    path_point.pose.position.x = point->position[0];
                    path_point.pose.position.y = point->position[1];
                    path_point.pose.position.z = point->position[2];
                    path_point.pose.orientation.w = 1.0;
                    mission_path_1.poses.push_back(path_point);
                }

                mission_point middle_start = *(exe_path1->end() - 1);
                mission_point before_start = *(exe_path1->end() - 15);
                mission_point after_start;

                after_start.position[0] = middle_start.position[0] + middle_start.position[0] - before_start.position[0];
                after_start.position[1] = middle_start.position[1] + middle_start.position[1] - before_start.position[1];
                after_start.position[2] = middle_start.position[2] + middle_start.position[2] - before_start.position[2];

                mission_point middle_end = *(exe_path2->end() - 1);
                mission_point before_end = *(exe_path2->end() - 15);
                mission_point after_end;

                after_end.position[0] = middle_end.position[0] + middle_end.position[0] - before_end.position[0];
                after_end.position[1] = middle_end.position[1] + middle_end.position[1] - before_end.position[1];
                after_end.position[2] = middle_end.position[2] + middle_end.position[2] - before_end.position[2];

                mission_point B1[4] = {middle_start, after_start, after_end, middle_end};//get control point

                auto test_point = get_point_by_B_and_u(B1, 0.5);//get 0.01 point;
                double test_distance = point_distance(middle_start.position, test_point.position);
                double delta = 0.5 * 5.0 / test_distance;

                for (double itt = delta; itt < 1.0; itt += delta)
                {
                    geometry_msgs::PoseStamped path_point;
                    auto route_point = get_point_by_B_and_u(B1, itt);
                    path_point.pose.position.x = route_point.position[0];
                    path_point.pose.position.y = route_point.position[1];
                    path_point.pose.position.z = route_point.position[2];
                    path_point.pose.orientation.w = 1.0;
                    mission_path_1.poses.push_back(path_point);
                }

                for (auto point = exe_path2->rbegin(); point != exe_path2->rend(); ++point)
                {
                    geometry_msgs::PoseStamped path_point;
                    path_point.pose.position.x = point->position[0];
                    path_point.pose.position.y = point->position[1];
                    path_point.pose.position.z = point->position[2];
                    path_point.pose.orientation.w = 1.0;
                    mission_path_1.poses.push_back(path_point);
                }


                geometry_msgs::PoseStamped path_point;
                path_point.pose.position.x = end_by_pose[0];
                path_point.pose.position.y = end_by_pose[1];
                path_point.pose.position.z = end_by_pose[2] + 5.0;
                path_point.pose.orientation.w = 1.0;
                mission_path_1.poses.push_back(path_point);

                target_world_pos[0] = end_by_pose[0];
                target_world_pos[1] = end_by_pose[1];
                target_world_pos[2] = end_by_pose[2] + 5.0;

                exe_path_publisher.publish(mission_path_1);
                mission_step = 10031;
            }
            return;
        }
        if (mission_step == 10031)
        {
            if (point_distance(target_world_pos, measure_world_pos) < 2.0)
            {
                target_world_pos[0] = end_by_pose[0];
                target_world_pos[1] = end_by_pose[1];
                target_world_pos[2] = end_by_pose[2] + 1.0;

                nav_msgs::Path direct_path;
                direct_path.header.stamp = ros::Time::now();
                direct_path.header.frame_id = "map";

                geometry_msgs::PoseStamped path_point;

                path_point.pose.position.x = end_by_pose[0];
                path_point.pose.position.y = end_by_pose[1];
                path_point.pose.position.z = end_by_pose[2] + 5.0;
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                path_point.pose.position.x = target_world_pos[0];
                path_point.pose.position.y = target_world_pos[1];
                path_point.pose.position.z = target_world_pos[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);
                exe_path_publisher.publish(direct_path);
                mission_step = 1003;
            }
            return;
        }
        if (mission_step == 1003)
        {
            if (point_distance(target_world_pos, measure_world_pos) < 2.0)
            {
                target_world_pos[0] = end_pose[0];
                target_world_pos[1] = end_pose[1];
                target_world_pos[2] = end_pose[2] + 1.5;

                nav_msgs::Path direct_path;
                direct_path.header.stamp = ros::Time::now();
                direct_path.header.frame_id = "map";

                geometry_msgs::PoseStamped path_point;

                path_point.pose.position.x = end_by_pose[0];
                path_point.pose.position.y = end_by_pose[1];
                path_point.pose.position.z = end_by_pose[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                path_point.pose.position.x = target_world_pos[0];
                path_point.pose.position.y = target_world_pos[1];
                path_point.pose.position.z = target_world_pos[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);
                exe_path_publisher.publish(direct_path);
                mission_step = 1004;
            }
            return;
        }
        if (mission_step == 1004)
        {
            if (point_distance(target_world_pos, measure_world_pos) < 1.0)
            {
                target_world_pos[0] = end_by_pose[0];
                target_world_pos[1] = end_by_pose[1];
                target_world_pos[2] = end_by_pose[2] + 1.0;

                nav_msgs::Path direct_path;
                direct_path.header.stamp = ros::Time::now();
                direct_path.header.frame_id = "map";

                geometry_msgs::PoseStamped path_point;

                path_point.pose.position.x = end_pose[0];
                path_point.pose.position.y = end_pose[1];
                path_point.pose.position.z = end_pose[2] + 1.5;
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                // path_point.pose.position.x = end_by_pose[0];
                // path_point.pose.position.y = end_by_pose[1];
                // path_point.pose.position.z = end_by_pose[2] + 1.0;
                // path_point.pose.orientation.w = 1.0;
                // direct_path.poses.push_back(path_point);

                path_point.pose.position.x = target_world_pos[0];
                path_point.pose.position.y = target_world_pos[1];
                path_point.pose.position.z = target_world_pos[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);
                exe_path_publisher.publish(direct_path);
                mission_step = 1005;
            }
            return;
        }
        if (mission_step == 1005)
        {
            force_strong_power_mode = true;
            if (point_distance(target_world_pos, measure_world_pos) < 3.0)
            {
                target_world_pos[0] = end_by_pose[0];
                target_world_pos[1] = end_by_pose[1];
                target_world_pos[2] = 190.0;
                tf_cmd[0] = target_world_pos[0];
                tf_cmd[1] = target_world_pos[1];
                tf_cmd[2] = target_world_pos[2];
                tf_cmd[3] = 0.0;
                tf_cmd[4] = 0.0;
                tf_cmd[5] = 10.0;
                mission_step = 1006;
            }
            return;
        }
        if (mission_step == 1006)
        {
            static bool eg_rebooted = false;
            if (!eg_rebooted)
            {
                if (measure_world_pos[2] > end_by_pose[2] + 30.0)
                {
                    eg_rebooted = true;
                    ros::NodeHandle pnh;
                    std_msgs::Float32 limit_msg;
                    pnh.setParam("/drone_0_ego_planner_node/manager/max_acc", 8.0);
                    pnh.setParam("/drone_0_ego_planner_node/manager/max_vel", 30.0);
                    pnh.setParam("/drone_0_ego_planner_node/manager/polyTraj_piece_length", 8.0);
                    pnh.setParam("/drone_0_ego_planner_node/optimization/max_acc", 8.0);
                    pnh.setParam("/drone_0_ego_planner_node/optimization/max_vel", 30.0);
                    limit_msg.data = 45.0;
                    planner_vel_limit_publisher.publish(limit_msg);
                }
            }

            tf_cmd[0] = target_world_pos[0];
            tf_cmd[1] = target_world_pos[1];
            tf_cmd[2] = target_world_pos[2];
            tf_cmd[3] = 0.0;
            tf_cmd[4] = 0.0;
            tf_cmd[5] = 0.0;

            if (measure_world_pos[2] > 180.0)
            {
                tf_cmd[5] = 0.0;
            }
            if (point_distance(target_world_pos, measure_world_pos) < 1.0)
            {
                mission_point back_end = *(exe_path1->end() - 1);
                mission_point after_end = *(exe_path1->end() - 60);
                mission_point berfore_end;

                berfore_end.position[0] = back_end.position[0] + back_end.position[0] - after_end.position[0];
                berfore_end.position[1] = back_end.position[1] + back_end.position[1] - after_end.position[1];
                berfore_end.position[2] = back_end.position[2] + back_end.position[2] - after_end.position[2];

                mission_point back_start;
                mission_point after_start;

                back_start.position[0] = end_by_pose[0];
                back_start.position[1] = end_by_pose[1];
                back_start.position[2] = 190.0;

                after_start.position[0] = back_end.position[0] - back_start.position[0];
                after_start.position[1] = back_end.position[1] - back_start.position[1];
                after_start.position[2] = back_end.position[2] - back_start.position[2];

                double length = sqrt(after_start.position[0] * after_start.position[0]
                    +after_start.position[1]*after_start.position[1] +
                        after_start.position[2]*after_start.position[2]);

                after_start.position[0] = after_start.position[0] / length * 30.0 +back_start.position[0];
                after_start.position[1] = after_start.position[1] / length * 30.0 +back_start.position[1];
                after_start.position[2] = after_start.position[2] / length * 30.0 +back_start.position[2];

                mission_point B1[4] = {back_start, after_start, berfore_end, back_end};//get control point

                auto test_point = get_point_by_B_and_u(B1, 0.5);//get 0.01 point;
                double test_distance = point_distance(back_start.position, test_point.position);
                double delta = 0.5 * 10.0 / test_distance;

                for (double itt = 0.0; itt < 1.0; itt += delta)
                {
                    geometry_msgs::PoseStamped path_point;
                    auto route_point = get_point_by_B_and_u(B1, itt);
                    path_point.pose.position.x = route_point.position[0];
                    path_point.pose.position.y = route_point.position[1];
                    path_point.pose.position.z = route_point.position[2];
                    path_point.pose.orientation.w = 1.0;
                    mission_path_2.poses.push_back(path_point);
                }

                for (auto point = exe_path1->rbegin(); point != exe_path1->rend(); ++point)
                {
                    geometry_msgs::PoseStamped path_point;
                    path_point.pose.position.x = point->position[0];
                    path_point.pose.position.y = point->position[1];
                    path_point.pose.position.z = point->position[2];
                    path_point.pose.orientation.w = 1.0;
                    mission_path_2.poses.push_back(path_point);
                }


                geometry_msgs::PoseStamped path_point;
                path_point.pose.position.x = start_by_pose[0];
                path_point.pose.position.y = start_by_pose[1];
                path_point.pose.position.z = start_by_pose[2] + 5.0;
                path_point.pose.orientation.w = 1.0;
                mission_path_2.poses.push_back(path_point);

                target_world_pos[0] = start_by_pose[0];
                target_world_pos[1] = start_by_pose[1];
                target_world_pos[2] = start_by_pose[2] + 5.0;
                mission_path_2.header.frame_id = "map";
                exe_path_publisher.publish(mission_path_2);

                mission_step = 1007;
            }
            return;
        }
        if (mission_step == 1007)
        {
            if (point_distance(target_world_pos, measure_world_pos) < 1.0)
            {
                target_world_pos[0] = start_by_pose[0];
                target_world_pos[1] = start_by_pose[1];
                target_world_pos[2] = start_by_pose[2] + 1.0;

                nav_msgs::Path direct_path;
                direct_path.header.stamp = ros::Time::now();
                direct_path.header.frame_id = "map";

                geometry_msgs::PoseStamped path_point;

                path_point.pose.position.x = start_by_pose[0];
                path_point.pose.position.y = start_by_pose[1];
                path_point.pose.position.z = start_by_pose[2] + 5.0;
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                path_point.pose.position.x = target_world_pos[0];
                path_point.pose.position.y = target_world_pos[1];
                path_point.pose.position.z = target_world_pos[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);
                exe_path_publisher.publish(direct_path);
                mission_step = 1008;
            }
            return;
        }
        if (mission_step == 1008)
        {
            if (point_distance(target_world_pos, measure_world_pos) < 1.0)
            {
                target_world_pos[0] = start_pose[0];
                target_world_pos[1] = start_pose[1];
                target_world_pos[2] = start_pose[2] + 1.5;

                nav_msgs::Path direct_path;
                direct_path.header.stamp = ros::Time::now();
                direct_path.header.frame_id = "map";

                geometry_msgs::PoseStamped path_point;

                path_point.pose.position.x = start_by_pose[0];
                path_point.pose.position.y = start_by_pose[1];
                path_point.pose.position.z = start_by_pose[2] + 1.0;
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);

                path_point.pose.position.x = target_world_pos[0];
                path_point.pose.position.y = target_world_pos[1];
                path_point.pose.position.z = target_world_pos[2];
                path_point.pose.orientation.w = 1.0;
                direct_path.poses.push_back(path_point);
                exe_path_publisher.publish(direct_path);
                mission_step = 1009;
            }
            return;
        }

    }
}
