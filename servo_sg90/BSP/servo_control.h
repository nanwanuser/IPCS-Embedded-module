/**
  ******************************************************************************
  * @file    servo_control.h
  * @brief   通用舵机控制模块头文件
  * @version V1.0.0
  * @date    2025-01-15
  ******************************************************************************
  */

#ifndef __SERVO_CONTROL_H
#define __SERVO_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"
/* ========================= 用户配置参数区 ========================= */
/**
  * @brief  舵机参数配置
  * @note   用户可根据实际舵机型号修改以下参数
  */
#define SERVO_PWM_PERIOD      20000   // PWM周期(us)，标准值20ms
#define SERVO_MIN_PULSE       500     // 最小脉宽(us)，对应0°
#define SERVO_MAX_PULSE       2500    // 最大脉宽(us)，对应180°
#define SERVO_DEFAULT_SPEED   100     // 默认转动速度(°/s)
#define SERVO_MAX_NUM         8       // 最大支持舵机数量

/* ========================= 数据类型定义 ========================= */
/**
  * @brief  舵机结构体
  */
typedef struct {
    uint8_t id;                       // 舵机ID
    float current_angle;              // 当前角度
    float target_angle;               // 目标角度
    float speed;                      // 转动速度(°/s)
    uint32_t last_update_time;        // 上次更新时间(ms)
    uint8_t is_moving;                // 是否正在转动
} ServoInfo;

/* ========================= API函数接口 ========================= */
/**
  * @brief  初始化舵机控制模块
  * @param  无
  * @retval 无
  */
void servo_init(void);

/**
  * @brief  设置舵机角度(立即转动)
  * @param  id    : 舵机ID (0 ~ SERVO_MAX_NUM-1)
  * @param  angle : 目标角度 (0.0 ~ 180.0)
  * @retval 0-成功, -1-参数错误
  */
int8_t servo_set_angle(uint8_t id, float angle);

/**
  * @brief  设置舵机角度(指定速度转动)
  * @param  id    : 舵机ID (0 ~ SERVO_MAX_NUM-1)
  * @param  angle : 目标角度 (0.0 ~ 180.0)
  * @param  speed : 转动速度 (°/s)
  * @retval 0-成功, -1-参数错误
  */
int8_t servo_set_angle_speed(uint8_t id, float angle, float speed);

/**
  * @brief  获取舵机当前角度
  * @param  id : 舵机ID (0 ~ SERVO_MAX_NUM-1)
  * @retval 当前角度值，错误返回-1
  */
float servo_get_angle(uint8_t id);

/**
  * @brief  更新舵机状态(需要在主循环中定期调用)
  * @param  无
  * @retval 无
  * @note   用于实现速度控制功能
  */
void servo_update(void);

/**
  * @brief  停止指定舵机
  * @param  id : 舵机ID (0 ~ SERVO_MAX_NUM-1)
  * @retval 0-成功, -1-参数错误
  */
int8_t servo_stop(uint8_t id);

/* ========================= 用户实现接口 ========================= */
/**
  * @brief  设置PWM输出(用户必须实现)
  * @param  channel : PWM通道号
  * @param  pulse   : 脉宽时间(us)
  * @retval 无
  * @note   用户需根据实际硬件定时器实现此函数
  */
void user_pwm_set_pulse(uint8_t channel, uint16_t pulse);

/**
  * @brief  获取系统时间(用户必须实现)
  * @param  无
  * @retval 系统运行时间(ms)
  * @note   用于速度控制计时
  */
uint32_t user_get_tick(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_CONTROL_H */