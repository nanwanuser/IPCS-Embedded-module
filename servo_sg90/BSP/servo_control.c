/**
  ******************************************************************************
  * @file    servo_control.c
  * @brief   通用舵机控制模块实现
  * @version V1.0.0
  * @date    2025-01-15
  ******************************************************************************
  */

#include "servo_control.h"
#include <string.h>
#include <math.h>

/* ========================= 私有变量 ========================= */
static ServoInfo g_servos[SERVO_MAX_NUM];

/* ========================= 私有函数 ========================= */
/**
  * @brief  角度转换为脉宽
  * @param  angle : 角度值(0-180°)
  * @retval 脉宽值(us)
  */
static uint16_t angle_to_pulse(float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    return (uint16_t)(SERVO_MIN_PULSE +
                     (angle / 180.0f) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
}

/**
  * @brief  限制角度范围
  * @param  angle : 输入角度
  * @retval 限制后的角度(0-180)
  */
static float limit_angle(float angle)
{
    if (angle < 0) return 0;
    if (angle > 180) return 180;
    return angle;
}

/* ========================= API函数实现 ========================= */
/**
  * @brief  初始化舵机控制模块
  * @param  无
  * @retval 无
  */
void servo_init(void)
{
    memset(g_servos, 0, sizeof(g_servos));

    /* 初始化每个舵机 */
    for (uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        g_servos[i].id = i;
        g_servos[i].current_angle = 90.0f;  // 默认中位
        g_servos[i].target_angle = 90.0f;
        g_servos[i].speed = SERVO_DEFAULT_SPEED;
        g_servos[i].is_moving = 0;

        /* 设置初始位置 */
        user_pwm_set_pulse(i, angle_to_pulse(90.0f));
    }
}

/**
  * @brief  设置舵机角度(立即转动)
  * @param  id    : 舵机ID
  * @param  angle : 目标角度
  * @retval 0-成功, -1-参数错误
  */
int8_t servo_set_angle(uint8_t id, float angle)
{
    if (id >= SERVO_MAX_NUM) {
        return -1;
    }

    angle = limit_angle(angle);

    g_servos[id].current_angle = angle;
    g_servos[id].target_angle = angle;
    g_servos[id].is_moving = 0;

    /* 立即输出PWM */
    user_pwm_set_pulse(id, angle_to_pulse(angle));

    return 0;
}

/**
  * @brief  设置舵机角度(指定速度转动)
  * @param  id    : 舵机ID
  * @param  angle : 目标角度
  * @param  speed : 转动速度
  * @retval 0-成功, -1-参数错误
  */
int8_t servo_set_angle_speed(uint8_t id, float angle, float speed)
{
    if (id >= SERVO_MAX_NUM || speed <= 0) {
        return -1;
    }

    angle = limit_angle(angle);

    g_servos[id].target_angle = angle;
    g_servos[id].speed = speed;
    g_servos[id].last_update_time = user_get_tick();
    g_servos[id].is_moving = 1;

    return 0;
}

/**
  * @brief  获取舵机当前角度
  * @param  id : 舵机ID
  * @retval 当前角度值，错误返回-1
  */
float servo_get_angle(uint8_t id)
{
    if (id >= SERVO_MAX_NUM) {
        return -1;
    }

    return g_servos[id].current_angle;
}

/**
  * @brief  更新舵机状态
  * @param  无
  * @retval 无
  */
void servo_update(void)
{
    uint32_t current_time = user_get_tick();

    for (uint8_t i = 0; i < SERVO_MAX_NUM; i++) {
        if (!g_servos[i].is_moving) {
            continue;
        }

        /* 计算时间差 */
        uint32_t delta_time = current_time - g_servos[i].last_update_time;
        g_servos[i].last_update_time = current_time;

        /* 计算角度变化 */
        float max_delta = g_servos[i].speed * delta_time / 1000.0f;
        float angle_diff = g_servos[i].target_angle - g_servos[i].current_angle;

        if (fabs(angle_diff) <= max_delta) {
            /* 到达目标角度 */
            g_servos[i].current_angle = g_servos[i].target_angle;
            g_servos[i].is_moving = 0;
        } else {
            /* 继续移动 */
            if (angle_diff > 0) {
                g_servos[i].current_angle += max_delta;
            } else {
                g_servos[i].current_angle -= max_delta;
            }
        }

        /* 更新PWM输出 */
        user_pwm_set_pulse(i, angle_to_pulse(g_servos[i].current_angle));
    }
}

/**
  * @brief  停止指定舵机
  * @param  id : 舵机ID
  * @retval 0-成功, -1-参数错误
  */
int8_t servo_stop(uint8_t id)
{
    if (id >= SERVO_MAX_NUM) {
        return -1;
    }

    g_servos[id].target_angle = g_servos[id].current_angle;
    g_servos[id].is_moving = 0;

    return 0;
}

/* ========================= 用户需要实现的函数 ========================= */
/**
  * @brief  设置PWM输出(用户必须实现)
  * @param  channel : PWM通道号
  * @param  pulse   : 脉宽时间(us)
  * @retval 无
  * @note   示例：TIM3->CCR1 = pulse * 72 / 1; // 72MHz主频，1分频
  */
void user_pwm_set_pulse(uint8_t channel, uint16_t pulse)
{
    /* 此函数需要用户实现 */
}

/**
  * @brief  获取系统时间(用户必须实现)
  * @param  无
  * @retval 系统运行时间(ms)
  * @note   示例：return HAL_GetTick();
  */
uint32_t user_get_tick(void)
{
    /* 此函数需要用户实现 */
    /*此处已有实现，可根据实际情况修改*/
    return HAL_GetTick();
}