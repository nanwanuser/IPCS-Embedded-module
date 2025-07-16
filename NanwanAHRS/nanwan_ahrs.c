/**
  ******************************************************************************
  * @file    nanwan_ahrs.c
  * @brief   姿态估计模块实现
  * @version V1.0.0
  * @date    2025-01-15
  ******************************************************************************
  */

#include "nanwan_ahrs.h"
#include <math.h>
#include <string.h>

/* ========================= 私有类型定义 ========================= */
/**
  * @brief  AHRS内部状态结构体
  */
typedef struct {
    float q0, q1, q2, q3;     // 四元数
    float sample_freq;        // 采样频率
    float beta;               // 收敛速度参数
    float dt;                 // 采样周期
    uint8_t use_mag;          // 是否使用磁力计
    uint8_t initialized;      // 初始化标志
} AhrsState;

/* ========================= 私有宏定义 ========================= */
#define RAD_TO_DEG(rad)  ((rad) * 180.0f / M_PI)  // 弧度转度数

/* ========================= 私有变量 ========================= */
static AhrsState g_state = {
    .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f,
    .sample_freq = AHRS_SAMPLE_FREQ,
    .beta = AHRS_BETA_DEFAULT,
    .dt = 1.0f / AHRS_SAMPLE_FREQ,
    .use_mag = AHRS_USE_MAGNETOMETER,
    .initialized = 0
};

/* ========================= 私有函数 ========================= */
/**
  * @brief  快速反平方根算法
  * @param  x : 输入值
  * @retval float : 1/sqrt(x)
  */
static float inv_sqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));  // 二次迭代提高精度
    return y;
}

/**
  * @brief  四元数归一化
  * @param  q : 四元数数组
  * @retval 无
  */
static void quaternion_normalize(float q[4])
{
    float norm = inv_sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    q[0] *= norm;
    q[1] *= norm;
    q[2] *= norm;
    q[3] *= norm;
}

/**
  * @brief  计算四元数微分
  * @param  q     : 当前四元数
  * @param  omega : 角速度
  * @param  qdot  : 输出的四元数微分
  * @retval 无
  */
static void quaternion_derivative(const float q[4], const float omega[3], float qdot[4])
{
    qdot[0] = 0.5f * (-q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2]);
    qdot[1] = 0.5f * ( q[0]*omega[0] + q[2]*omega[2] - q[3]*omega[1]);
    qdot[2] = 0.5f * ( q[0]*omega[1] - q[1]*omega[2] + q[3]*omega[0]);
    qdot[3] = 0.5f * ( q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]);
}

/**
  * @brief  四阶龙格-库塔积分
  * @param  q     : 当前四元数
  * @param  omega : 角速度
  * @param  dt    : 时间步长
  * @param  q_out : 输出的新四元数
  * @retval 无
  */
static void runge_kutta4_integration(const float q[4], const float omega[3], float dt, float q_out[4])
{
    float k1[4], k2[4], k3[4], k4[4];
    float q_temp[4];
    int i;
    
    // k1 = f(q, omega)
    quaternion_derivative(q, omega, k1);
    
    // k2 = f(q + dt/2 * k1, omega)
    for (i = 0; i < 4; i++) {
        q_temp[i] = q[i] + 0.5f * dt * k1[i];
    }
    quaternion_derivative(q_temp, omega, k2);
    
    // k3 = f(q + dt/2 * k2, omega)
    for (i = 0; i < 4; i++) {
        q_temp[i] = q[i] + 0.5f * dt * k2[i];
    }
    quaternion_derivative(q_temp, omega, k3);
    
    // k4 = f(q + dt * k3, omega)
    for (i = 0; i < 4; i++) {
        q_temp[i] = q[i] + dt * k3[i];
    }
    quaternion_derivative(q_temp, omega, k4);
    
    // q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    for (i = 0; i < 4; i++) {
        q_out[i] = q[i] + (dt / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);
    }
    
    // 归一化
    quaternion_normalize(q_out);
}

/**
  * @brief  计算梯度下降步长（9轴）
  * @param  q : 当前四元数
  * @param  a : 加速度计数据（归一化）
  * @param  m : 磁力计数据（归一化）
  * @param  gradient : 输出的梯度
  * @retval 无
  */
static void compute_gradient_9axis(const float q[4], const float a[3], const float m[3], float gradient[4])
{
    float hx, hy, bx, bz;
    float vx, vy, vz;
    float wx, wy, wz;
    
    // 辅助变量
    float q0q0 = q[0] * q[0];
    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q0q3 = q[0] * q[3];
    float q1q1 = q[1] * q[1];
    float q1q2 = q[1] * q[2];
    float q1q3 = q[1] * q[3];
    float q2q2 = q[2] * q[2];
    float q2q3 = q[2] * q[3];
    float q3q3 = q[3] * q[3];
    
    // 计算地磁场参考方向
    hx = 2.0f * (m[0] * (0.5f - q2q2 - q3q3) + m[1] * (q1q2 - q0q3) + m[2] * (q1q3 + q0q2));
    hy = 2.0f * (m[0] * (q1q2 + q0q3) + m[1] * (0.5f - q1q1 - q3q3) + m[2] * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (m[0] * (q1q3 - q0q2) + m[1] * (q2q3 + q0q1) + m[2] * (0.5f - q1q1 - q2q2));
    
    // 计算目标函数的梯度
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
    
    // 计算梯度
    gradient[0] = -2.0f * q[2] * (vx - a[0]) + 2.0f * q[1] * (vy - a[1]) + 
                  -2.0f * bz * q[2] * (wx - m[0]) + 2.0f * bz * q[1] * (wy - m[1]) + 
                  2.0f * bx * q[3] * (wx - m[0]) + 2.0f * bx * q[0] * (wz - m[2]) - 
                  2.0f * bx * q[1] * (wy - m[1]);
                  
    gradient[1] = 2.0f * q[3] * (vx - a[0]) + 2.0f * q[0] * (vy - a[1]) - 
                  4.0f * q[1] * (vz - a[2]) + 2.0f * bz * q[3] * (wx - m[0]) + 
                  2.0f * bz * q[0] * (wy - m[1]) + 2.0f * bx * q[2] * (wx - m[0]) - 
                  2.0f * bx * q[0] * (wy - m[1]) + 2.0f * bx * q[1] * (wz - m[2]);
                  
    gradient[2] = -2.0f * q[0] * (vx - a[0]) + 2.0f * q[3] * (vy - a[1]) - 
                  4.0f * q[2] * (vz - a[2]) - 4.0f * bx * q[2] * (wx - m[0]) - 
                  2.0f * bz * q[0] * (wx - m[0]) + 2.0f * bz * q[3] * (wy - m[1]) + 
                  2.0f * bx * q[1] * (wx - m[0]) + 2.0f * bx * q[2] * (wz - m[2]);
                  
    gradient[3] = 2.0f * q[1] * (vx - a[0]) + 2.0f * q[2] * (vy - a[1]) + 
                  2.0f * bz * q[1] * (wx - m[0]) - 4.0f * bx * q[3] * (wx - m[0]) + 
                  2.0f * bz * q[2] * (wy - m[1]) - 2.0f * bx * q[0] * (wx - m[0]) + 
                  2.0f * bx * q[3] * (wz - m[2]);
                  
    // 归一化梯度
    float norm = inv_sqrt(gradient[0]*gradient[0] + gradient[1]*gradient[1] + 
                         gradient[2]*gradient[2] + gradient[3]*gradient[3]);
    gradient[0] *= norm;
    gradient[1] *= norm;
    gradient[2] *= norm;
    gradient[3] *= norm;
}

/**
  * @brief  计算梯度下降步长（6轴）
  * @param  q : 当前四元数
  * @param  a : 加速度计数据（归一化）
  * @param  gradient : 输出的梯度
  * @retval 无
  */
static void compute_gradient_6axis(const float q[4], const float a[3], float gradient[4])
{
    // 辅助变量
    float _2q0 = 2.0f * q[0];
    float _2q1 = 2.0f * q[1];
    float _2q2 = 2.0f * q[2];
    float _2q3 = 2.0f * q[3];
    float _4q0 = 4.0f * q[0];
    float _4q1 = 4.0f * q[1];
    float _4q2 = 4.0f * q[2];
    float _8q1 = 8.0f * q[1];
    float _8q2 = 8.0f * q[2];
    float q0q0 = q[0] * q[0];
    float q1q1 = q[1] * q[1];
    float q2q2 = q[2] * q[2];
    float q3q3 = q[3] * q[3];
    
    // 计算梯度
    gradient[0] = _4q0 * q2q2 + _2q2 * a[0] + _4q0 * q1q1 - _2q1 * a[1];
    gradient[1] = _4q1 * q3q3 - _2q3 * a[0] + 4.0f * q0q0 * q[1] - _2q0 * a[1] - 
                  _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a[2];
    gradient[2] = 4.0f * q0q0 * q[2] + _2q0 * a[0] + _4q2 * q3q3 - _2q3 * a[1] - 
                  _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a[2];
    gradient[3] = 4.0f * q1q1 * q[3] - _2q1 * a[0] + 4.0f * q2q2 * q[3] - _2q2 * a[1];
    
    // 归一化梯度
    float norm = inv_sqrt(gradient[0]*gradient[0] + gradient[1]*gradient[1] + 
                         gradient[2]*gradient[2] + gradient[3]*gradient[3]);
    gradient[0] *= norm;
    gradient[1] *= norm;
    gradient[2] *= norm;
    gradient[3] *= norm;
}

/* ========================= API函数实现 ========================= */
/**
  * @brief  初始化AHRS模块
  * @param  config : 配置参数指针（为NULL时使用默认配置）
  * @retval AhrsStatus : 初始化状态
  */
AhrsStatus nanwan_ahrs_init(AhrsConfig *config)
{
    if (config != NULL) {
        g_state.sample_freq = config->sample_freq;
        g_state.beta = config->beta;
        g_state.use_mag = config->use_mag;
        g_state.dt = 1.0f / config->sample_freq;
    } else {
        g_state.sample_freq = AHRS_SAMPLE_FREQ;
        g_state.beta = AHRS_BETA_DEFAULT;
        g_state.use_mag = AHRS_USE_MAGNETOMETER;
        g_state.dt = 1.0f / AHRS_SAMPLE_FREQ;
    }
    
    // 重置四元数
    g_state.q0 = 1.0f;
    g_state.q1 = 0.0f;
    g_state.q2 = 0.0f;
    g_state.q3 = 0.0f;
    
    g_state.initialized = 1;
    
    return AHRS_OK;
}

/**
  * @brief  更新姿态估计（使用9轴数据）
  * @param  sensor : 传感器数据指针
  * @retval AhrsStatus : 更新状态
  */
AhrsStatus nanwan_ahrs_update(AhrsSensorData *sensor)
{
    float norm;
    float gradient[4];
    float omega_corrected[3];
    float q_temp[4];
    
    if (!g_state.initialized || sensor == NULL) {
        return AHRS_NOT_INITIALIZED;
    }
    
    // 检查磁力计数据，如果全为0则使用6轴模式
    if ((sensor->mx == 0.0f) && (sensor->my == 0.0f) && (sensor->mz == 0.0f)) {
        return nanwan_ahrs_update_imu(sensor->gx, sensor->gy, sensor->gz,
                                      sensor->ax, sensor->ay, sensor->az);
    }
    
    // 归一化加速度计数据
    norm = inv_sqrt(sensor->ax * sensor->ax + sensor->ay * sensor->ay + sensor->az * sensor->az);
    float ax = sensor->ax * norm;
    float ay = sensor->ay * norm;
    float az = sensor->az * norm;
    
    // 归一化磁力计数据
    norm = inv_sqrt(sensor->mx * sensor->mx + sensor->my * sensor->my + sensor->mz * sensor->mz);
    float mx = sensor->mx * norm;
    float my = sensor->my * norm;
    float mz = sensor->mz * norm;
    
    // 计算梯度
    float a[3] = {ax, ay, az};
    float m[3] = {mx, my, mz};
    float q[4] = {g_state.q0, g_state.q1, g_state.q2, g_state.q3};
    compute_gradient_9axis(q, a, m, gradient);
    
    // 应用反馈
    omega_corrected[0] = sensor->gx - g_state.beta * gradient[1];
    omega_corrected[1] = sensor->gy - g_state.beta * gradient[2];
    omega_corrected[2] = sensor->gz - g_state.beta * gradient[3];
    
    // 四阶龙格-库塔积分更新四元数
    runge_kutta4_integration(q, omega_corrected, g_state.dt, q_temp);
    
    // 更新状态
    g_state.q0 = q_temp[0];
    g_state.q1 = q_temp[1];
    g_state.q2 = q_temp[2];
    g_state.q3 = q_temp[3];
    
    return AHRS_OK;
}

/**
  * @brief  更新姿态估计（仅使用6轴数据）
  * @param  gx,gy,gz : 陀螺仪数据（rad/s）
  * @param  ax,ay,az : 加速度计数据（m/s²或g）
  * @retval AhrsStatus : 更新状态
  */
AhrsStatus nanwan_ahrs_update_imu(float gx, float gy, float gz, 
                                   float ax, float ay, float az)
{
    float norm;
    float gradient[4];
    float omega_corrected[3];
    float q_temp[4];
    
    if (!g_state.initialized) {
        return AHRS_NOT_INITIALIZED;
    }
    
    // 如果加速度计数据全为0，仅使用陀螺仪积分
    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) {
        float omega[3] = {gx, gy, gz};
        float q[4] = {g_state.q0, g_state.q1, g_state.q2, g_state.q3};
        runge_kutta4_integration(q, omega, g_state.dt, q_temp);
        
        g_state.q0 = q_temp[0];
        g_state.q1 = q_temp[1];
        g_state.q2 = q_temp[2];
        g_state.q3 = q_temp[3];
        
        return AHRS_OK;
    }
    
    // 归一化加速度计数据
    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    // 计算梯度
    float a[3] = {ax, ay, az};
    float q[4] = {g_state.q0, g_state.q1, g_state.q2, g_state.q3};
    compute_gradient_6axis(q, a, gradient);
    
    // 应用反馈
    omega_corrected[0] = gx - g_state.beta * gradient[1];
    omega_corrected[1] = gy - g_state.beta * gradient[2];
    omega_corrected[2] = gz - g_state.beta * gradient[3];
    
    // 四阶龙格-库塔积分更新四元数
    runge_kutta4_integration(q, omega_corrected, g_state.dt, q_temp);
    
    // 更新状态
    g_state.q0 = q_temp[0];
    g_state.q1 = q_temp[1];
    g_state.q2 = q_temp[2];
    g_state.q3 = q_temp[3];
    
    return AHRS_OK;
}

/**
  * @brief  获取当前姿态
  * @param  attitude : 姿态数据指针
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_attitude(AhrsAttitude *attitude)
{
    if (!g_state.initialized || attitude == NULL) {
        return AHRS_NOT_INITIALIZED;
    }
    
    // 保存四元数
    attitude->q0 = g_state.q0;
    attitude->q1 = g_state.q1;
    attitude->q2 = g_state.q2;
    attitude->q3 = g_state.q3;
    
    // 计算欧拉角
    attitude->roll = atan2f(2.0f * (g_state.q0 * g_state.q1 + g_state.q2 * g_state.q3),
                           1.0f - 2.0f * (g_state.q1 * g_state.q1 + g_state.q2 * g_state.q2));
    
    float sinp = 2.0f * (g_state.q0 * g_state.q2 - g_state.q3 * g_state.q1);
    if (fabsf(sinp) >= 1.0f) {
        attitude->pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        attitude->pitch = asinf(sinp);
    }
    
    attitude->yaw = atan2f(2.0f * (g_state.q0 * g_state.q3 + g_state.q1 * g_state.q2),
                          1.0f - 2.0f * (g_state.q2 * g_state.q2 + g_state.q3 * g_state.q3));
    
    return AHRS_OK;
}

/**
  * @brief  获取四元数
  * @param  q : 四元数数组指针
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_quaternion(float q[4])
{
    if (!g_state.initialized || q == NULL) {
        return AHRS_NOT_INITIALIZED;
    }
    
    q[0] = g_state.q0;
    q[1] = g_state.q1;
    q[2] = g_state.q2;
    q[3] = g_state.q3;
    
    return AHRS_OK;
}

/**
  * @brief  获取欧拉角
  * @param  roll  : 横滚角指针（rad）
  * @param  pitch : 俯仰角指针（rad）
  * @param  yaw   : 偏航角指针（rad）
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_euler(float *roll, float *pitch, float *yaw)
{
    if (!g_state.initialized || roll == NULL || pitch == NULL || yaw == NULL) {
        return AHRS_NOT_INITIALIZED;
    }
    
    *roll = atan2f(2.0f * (g_state.q0 * g_state.q1 + g_state.q2 * g_state.q3),
                   1.0f - 2.0f * (g_state.q1 * g_state.q1 + g_state.q2 * g_state.q2));
    
    float sinp = 2.0f * (g_state.q0 * g_state.q2 - g_state.q3 * g_state.q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = copysignf(M_PI / 2.0f, sinp);
    } else {
        *pitch = asinf(sinp);
    }
    
    *yaw = atan2f(2.0f * (g_state.q0 * g_state.q3 + g_state.q1 * g_state.q2),
                  1.0f - 2.0f * (g_state.q2 * g_state.q2 + g_state.q3 * g_state.q3));
    
    return AHRS_OK;
}

/**
  * @brief  重置姿态
  * @param  无
  * @retval AhrsStatus : 重置状态
  */
AhrsStatus nanwan_ahrs_reset(void)
{
    g_state.q0 = 1.0f;
    g_state.q1 = 0.0f;
    g_state.q2 = 0.0f;
    g_state.q3 = 0.0f;
    
    return AHRS_OK;
}

/**
  * @brief  设置收敛速度参数
  * @param  beta : 收敛速度参数
  * @retval AhrsStatus : 设置状态
  */
AhrsStatus nanwan_ahrs_set_beta(float beta)
{
    if (beta < 0.0f || beta > 1.0f) {
        return AHRS_ERROR;
    }
    
    g_state.beta = beta;
    
    return AHRS_OK;
}

/**
  * @brief  获取当前配置
  * @param  config : 配置结构体指针
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_config(AhrsConfig *config)
{
    if (config == NULL) {
        return AHRS_ERROR;
    }
    
    config->sample_freq = g_state.sample_freq;
    config->beta = g_state.beta;
    config->use_mag = g_state.use_mag;
    
    return AHRS_OK;
}

/**
  * @brief  获取欧拉角（度数）
  * @param  roll  : 横滚角指针（deg）
  * @param  pitch : 俯仰角指针（deg）
  * @param  yaw   : 偏航角指针（deg）
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_euler_degrees(float *roll, float *pitch, float *yaw)
{
    if (!g_state.initialized || roll == NULL || pitch == NULL || yaw == NULL) {
        return AHRS_NOT_INITIALIZED;
    }
    
    // 直接计算并转换为度数
    *roll = RAD_TO_DEG(atan2f(2.0f * (g_state.q0 * g_state.q1 + g_state.q2 * g_state.q3),
                              1.0f - 2.0f * (g_state.q1 * g_state.q1 + g_state.q2 * g_state.q2)));
    
    float sinp = 2.0f * (g_state.q0 * g_state.q2 - g_state.q3 * g_state.q1);
    if (fabsf(sinp) >= 1.0f) {
        *pitch = RAD_TO_DEG(copysignf(M_PI / 2.0f, sinp));
    } else {
        *pitch = RAD_TO_DEG(asinf(sinp));
    }
    
    *yaw = RAD_TO_DEG(atan2f(2.0f * (g_state.q0 * g_state.q3 + g_state.q1 * g_state.q2),
                             1.0f - 2.0f * (g_state.q2 * g_state.q2 + g_state.q3 * g_state.q3)));
    
    return AHRS_OK;
}
