#include <stdint.h>
#include <stdbool.h>
#include "encoder_module.h"
#include "tim.h"
#include <string.h>

// =============================================================================
// 静态数据定义
// =============================================================================

// 编码器实例数组
static Encoder_Data_t encoder_instances[MAX_ENCODER_COUNT] = {0};
// 编码器数量计数器
static uint8_t encoder_count = 0;

// =============================================================================
// 内部辅助函数
// =============================================================================

/**
 * @brief  检查编码器句柄是否有效
 * @param  handle: 编码器句柄
 * @retval uint8_t 1=有效，0=无效
 */
static uint8_t is_handle_valid(encoder_handle_t handle) {
    return (handle < MAX_ENCODER_COUNT && encoder_instances[handle].htim != NULL);
}

/**
 * @brief  获取编码器数据指针
 * @param  handle: 编码器句柄
 * @retval Encoder_Data_t* 编码器数据指针，失败返回NULL
 */
static Encoder_Data_t* get_encoder_data(encoder_handle_t handle) {
    if (!is_handle_valid(handle)) {
        return NULL;
    }
    return &encoder_instances[handle];
}

// =============================================================================
// 编码器管理函数
// =============================================================================

/**
 * @brief  创建编码器实例
 * @param  htim: 定时器句柄指针
 * @retval encoder_handle_t 编码器句柄，失败返回0xFF
 * @note   创建一个编码器实例并分配句柄，但不初始化硬件
 */
encoder_handle_t Encoder_Create(TIM_HandleTypeDef* htim) {
    if (htim == NULL || encoder_count >= MAX_ENCODER_COUNT) {
        return 0xFF; // 无效句柄
    }

    // 寻找空闲槽位
    for (uint8_t i = 0; i < MAX_ENCODER_COUNT; i++) {
        if (encoder_instances[i].htim == NULL) {
            // 初始化编码器数据结构
            memset(&encoder_instances[i], 0, sizeof(Encoder_Data_t));
            encoder_instances[i].htim = htim;
            encoder_instances[i].initialized = 0;
            encoder_count++;
            return i;
        }
    }

    return 0xFF; // 无可用槽位
}

/**
 * @brief  编码器初始化
 * @param  handle: 编码器句柄
 * @retval Motor_Result_t 初始化结果，MOTOR_OK 或 MOTOR_HARDWARE_ERROR
 * @note   启动定时器编码器接口，设置初值，初始化数据结构体
 */
Motor_Result_t Encoder_Init(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL) {
        return MOTOR_HARDWARE_ERROR;
    }

    // 启动定时器编码器接口
    if (HAL_TIM_Encoder_Start(data->htim, TIM_CHANNEL_ALL) != HAL_OK) {
        return MOTOR_HARDWARE_ERROR;
    }

    // 将定时器计数器设置为中点（便于正负溢出判断）
    __HAL_TIM_SET_COUNTER(data->htim, ENCODER_COUNTER_MIDPOINT);

    // 初始化编码器数据
    data->total_count = 0;
    data->last_count = ENCODER_COUNTER_MIDPOINT;
    data->last_count_for_speed = ENCODER_COUNTER_MIDPOINT;
    data->speed_rpm = 0;
    data->buffer_index = 0;
    data->last_update_time = HAL_GetTick();
    data->position_mm = 0.0f;
    data->last_position_mm = 0.0f;

    // 初始化速度滤波缓冲区
    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        data->speed_buffer[i] = 0;
    }

    data->initialized = 1;
    return MOTOR_OK;
}

/**
 * @brief  销毁编码器实例
 * @param  handle: 编码器句柄
 * @note   停止定时器，清空数据结构，释放句柄
 */
void Encoder_Destroy(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL) {
        return;
    }

    // 停止定时器编码器接口
    if (data->initialized) {
        HAL_TIM_Encoder_Stop(data->htim, TIM_CHANNEL_ALL);
    }

    // 清空数据结构
    memset(data, 0, sizeof(Encoder_Data_t));
    encoder_count--;
}

// =============================================================================
// 编码器数据获取函数
// =============================================================================

/**
 * @brief  获取编码器累计计数值
 * @param  handle: 编码器句柄
 * @retval int32_t 累计计数值
 * @note   处理定时器溢出，返回累计脉冲数
 */
int32_t Encoder_GetCount(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL || !data->initialized) {
        return 0;
    }

    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(data->htim);
    int32_t delta = current_count - data->last_count;

    // 处理定时器溢出（正溢出和负溢出）
    if (delta > ENCODER_OVERFLOW_THRESHOLD) {
        delta -= ENCODER_OVERFLOW_RANGE;
    } else if (delta < -ENCODER_OVERFLOW_THRESHOLD) {
        delta += ENCODER_OVERFLOW_RANGE;
    }

    data->total_count += delta;
    data->last_count = current_count;

    return data->total_count;
}

/**
 * @brief  获取编码器转速（单位：RPM）
 * @param  handle: 编码器句柄
 * @retval int16_t 转速（RPM）
 * @note   采用滤波缓冲区做平均，采样周期由ENCODER_SAMPLE_TIME控制
 */
int16_t Encoder_GetSpeed(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL || !data->initialized) {
        return 0; // 如果未初始化，直接返回0
    }

    unsigned long current_time = HAL_GetTick(); // 获取当前时间（ms）
    unsigned long time_diff = current_time - data->last_update_time; // 距离上次采样的时间差

    if (time_diff < ENCODER_SAMPLE_TIME) {
        return data->speed_rpm; // 采样周期未到，直接返回上次速度
    }

    int32_t last_count = data->last_count_for_speed; // 上一次的计数器值
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(data->htim); // 当前计数器值
    int32_t delta = current_count - last_count; // 计算本周期内的脉冲数变化

    // 处理定时器溢出（正溢出和负溢出）
    if (delta > ENCODER_OVERFLOW_THRESHOLD) delta -= ENCODER_OVERFLOW_RANGE;
    else if (delta < -ENCODER_OVERFLOW_THRESHOLD) delta += ENCODER_OVERFLOW_RANGE;

    // 计算转速（RPM）：脉冲数变化/每转脉冲数*每分钟ms数/时间差
    float speed_rpm = (float)delta / (ENCODER_PPR * ENCODER_QUADRATURE) * (60000.0f / time_diff);

    data->last_count_for_speed = current_count;
    data->last_update_time = current_time; // 更新时间戳
    data->speed_rpm = (int16_t)speed_rpm; // 保存当前转速

    return data->speed_rpm; // 返回当前转速
}

/**
 * @brief  获取编码器累计位置（单位：mm）
 * @param  handle: 编码器句柄
 * @retval float 累计位置（mm）
 * @note   根据累计计数计算实际位移，单位mm
 */
float Encoder_GetPosition(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL || !data->initialized) {
        return 0.0f;
    }

    int32_t count = Encoder_GetCount(handle);
    // 位置 = (累计计数 / (PPR * 4)) * 2π * 轮胎半径 * 传动比
    float position = (float)count / (ENCODER_PPR * ENCODER_QUADRATURE)
                     * 2.0f * 3.14159f * WHEEL_RADIUS * GEAR_RATIO / 1000.0f;
    data->position_mm = position;
    return position;
}

/**
 * @brief  获取编码器累计距离（单位：mm）
 * @param  handle: 编码器句柄
 * @retval float 累计距离（mm）
 * @note   计算当前位置与上次位置的差值
 */
float Encoder_GetDistance(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL || !data->initialized) {
        return 0.0f;
    }

    float current_position = Encoder_GetPosition(handle);
    float distance = current_position - data->last_position_mm;
    data->last_position_mm = current_position;
    return distance;
}

// =============================================================================
// 编码器控制函数
// =============================================================================

/**
 * @brief  编码器复位
 * @param  handle: 编码器句柄
 * @note   计数器和数据结构体全部清零
 */
void Encoder_Reset(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL || !data->initialized) {
        return;
    }

    __HAL_TIM_SET_COUNTER(data->htim, ENCODER_COUNTER_MIDPOINT);
    data->total_count = 0;
    data->last_count = ENCODER_COUNTER_MIDPOINT;
    data->last_count_for_speed = ENCODER_COUNTER_MIDPOINT;
    data->speed_rpm = 0;
    data->buffer_index = 0;
    data->last_update_time = HAL_GetTick();
    data->position_mm = 0.0f;
    data->last_position_mm = 0.0f;

    for (int i = 0; i < ENCODER_FILTER_SIZE; i++) {
        data->speed_buffer[i] = 0;
    }
}

/**
 * @brief  编码器校准
 * @param  handle: 编码器句柄
 * @retval Motor_Result_t 校准结果
 * @note   复位后检查计数器是否为0
 */
Motor_Result_t Encoder_Calibrate(encoder_handle_t handle) {
    Encoder_Data_t* data = get_encoder_data(handle);
    if (data == NULL || !data->initialized) {
        return MOTOR_HARDWARE_ERROR;
    }

    Encoder_Reset(handle);
    HAL_Delay(100);
    int32_t count = Encoder_GetCount(handle);
    if (count != 0) {
        return MOTOR_HARDWARE_ERROR;
    }
    return MOTOR_OK;
}

// =============================================================================
// 便捷函数
// =============================================================================

/**
 * @brief  快速初始化编码器
 * @param  htim: 定时器句柄指针
 * @param  handle: 编码器句柄指针（用于返回句柄）
 * @retval Motor_Result_t 初始化结果
 * @note   一步完成创建和初始化过程
 */
Motor_Result_t Encoder_QuickInit(TIM_HandleTypeDef* htim, encoder_handle_t* handle) {
    if (handle == NULL) {
        return MOTOR_HARDWARE_ERROR;
    }

    *handle = Encoder_Create(htim);
    if (*handle == 0xFF) {
        return MOTOR_HARDWARE_ERROR;
    }

    Motor_Result_t result = Encoder_Init(*handle);
    if (result != MOTOR_OK) {
        Encoder_Destroy(*handle);
        *handle = 0xFF;
    }

    return result;
}

/**
 * @brief  检查句柄是否有效
 * @param  handle: 编码器句柄
 * @retval uint8_t 1=有效，0=无效
 */
uint8_t Encoder_IsValid(encoder_handle_t handle) {
    return is_handle_valid(handle);
}