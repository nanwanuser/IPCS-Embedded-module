/**
  ******************************************************************************
  * @file    nanwan_ahrs.h
  * @brief   姿态估计模块头文件
  * @version V1.0.0
  * @date    2025-01-15
  ******************************************************************************
  */

#ifndef __NANWAN_AHRS_H
#define __NANWAN_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================= 用户配置参数区 ========================= */
/**
  * @brief  AHRS参数配置
  * @note   用户可根据实际需求修改以下参数
  */
#define AHRS_SAMPLE_FREQ     100.0f   // 采样频率（Hz）
#define AHRS_BETA_DEFAULT    0.1f     // 默认收敛速度参数（越大收敛越快但噪声越大）
#define AHRS_USE_MAGNETOMETER 1       // 是否使用磁力计（0-仅使用陀螺仪和加速度计）

/* ========================= 数据类型定义 ========================= */
// 常用数学常量
#ifndef M_PI
#define M_PI                  3.14159265358979323846f
#endif
/**
  * @brief  传感器数据结构体
  */
typedef struct {
    float gx, gy, gz;     // 陀螺仪数据（rad/s）
    float ax, ay, az;     // 加速度计数据（归一化）
    float mx, my, mz;     // 磁力计数据（归一化）
} AhrsSensorData;

/**
  * @brief  姿态数据结构体
  */
typedef struct {
    float roll;           // 横滚角（rad）
    float pitch;          // 俯仰角（rad）
    float yaw;            // 偏航角（rad）
    float q0, q1, q2, q3; // 四元数
} AhrsAttitude;

/**
  * @brief  AHRS配置结构体
  */
typedef struct {
    float sample_freq;    // 采样频率（Hz）
    float beta;           // 收敛速度参数
    uint8_t use_mag;      // 是否使用磁力计
} AhrsConfig;

/**
  * @brief  AHRS状态枚举
  */
typedef enum {
    AHRS_OK = 0,          // 正常
    AHRS_ERROR,           // 错误
    AHRS_NOT_INITIALIZED  // 未初始化
} AhrsStatus;

/* ========================= API函数接口 ========================= */
/**
  * @brief  初始化AHRS模块
  * @param  config : 配置参数指针（为NULL时使用默认配置）
  * @retval AhrsStatus : 初始化状态
  */
AhrsStatus nanwan_ahrs_init(AhrsConfig *config);

/**
  * @brief  更新姿态估计（使用9轴数据）
  * @param  sensor : 传感器数据指针
  * @retval AhrsStatus : 更新状态
  * @note   磁力计数据为0时自动切换到6轴模式
  */
AhrsStatus nanwan_ahrs_update(AhrsSensorData *sensor);

/**
  * @brief  更新姿态估计（仅使用6轴数据）
  * @param  gx,gy,gz : 陀螺仪数据（rad/s）
  * @param  ax,ay,az : 加速度计数据（m/s²或g）
  * @retval AhrsStatus : 更新状态
  * @note   加速度计数据会自动归一化
  */
AhrsStatus nanwan_ahrs_update_imu(float gx, float gy, float gz, 
                                   float ax, float ay, float az);

/**
  * @brief  获取当前姿态
  * @param  attitude : 姿态数据指针
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_attitude(AhrsAttitude *attitude);

/**
  * @brief  获取四元数
  * @param  q : 四元数数组指针（q[0]=q0, q[1]=q1, q[2]=q2, q[3]=q3）
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_quaternion(float q[4]);

/**
  * @brief  获取欧拉角
  * @param  roll  : 横滚角指针（rad）
  * @param  pitch : 俯仰角指针（rad）
  * @param  yaw   : 偏航角指针（rad）
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_euler(float *roll, float *pitch, float *yaw);

/**
  * @brief  获取欧拉角（度数）
  * @param  roll  : 横滚角指针（deg）
  * @param  pitch : 俯仰角指针（deg）
  * @param  yaw   : 偏航角指针（deg）
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_euler_degrees(float *roll, float *pitch, float *yaw);

/**
  * @brief  重置姿态
  * @param  无
  * @retval AhrsStatus : 重置状态
  * @note   将姿态重置为初始状态（四元数=[1,0,0,0]）
  */
AhrsStatus nanwan_ahrs_reset(void);

/**
  * @brief  设置收敛速度参数
  * @param  beta : 收敛速度参数（建议范围：0.01-0.5）
  * @retval AhrsStatus : 设置状态
  */
AhrsStatus nanwan_ahrs_set_beta(float beta);

/**
  * @brief  获取当前配置
  * @param  config : 配置结构体指针
  * @retval AhrsStatus : 获取状态
  */
AhrsStatus nanwan_ahrs_get_config(AhrsConfig *config);

#ifdef __cplusplus
}
#endif

#endif /* __NANWAN_AHRS_H */
