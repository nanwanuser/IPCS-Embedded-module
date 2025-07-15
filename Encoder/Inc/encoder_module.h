#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>    // 必须添加，保证标准类型可用
#include "main.h"

// =============================================================================
// 编码器配置参数定义
// =============================================================================

// MG370电机硬件参数定义（可根据实际电机调整）
#define ENCODER_PPR           13      // MG370霍尔编码器每圈脉冲数【13】（典型值，请根据实际型号确认）
#define ENCODER_QUADRATURE    4       // 四倍频
#define ENCODER_SAMPLE_TIME   20      // 转速采样周期(ms)
#define ENCODER_FILTER_SIZE   5       // 滤波缓冲区大小

// 机械参数定义（请根据实际硬件调整）
#define MOTOR_SHAFT_DIAMETER  4       // 电机轴径(mm)
#define WHEEL_RADIUS          2.4f    // 轮胎半径(mm)，请替换为实际值
#define GEAR_RATIO            34      // MG370霍尔编码器传动比【34】，请替换为实际值

// 定时器配置参数（可根据定时器位数调整）
#define ENCODER_COUNTER_MIDPOINT      32768   // 16位定时器中点值
#define ENCODER_OVERFLOW_THRESHOLD    32767   // 16位定时器溢出阈值
#define ENCODER_OVERFLOW_RANGE        65536   // 16位定时器量程

// 最大支持编码器数量
#define MAX_ENCODER_COUNT    4

// =============================================================================
// 数据结构定义
// =============================================================================

// 编码器数据结构体，保存编码器的计数、速度、位置等信息
typedef struct {
    int32_t total_count;                       // 累计脉冲数
    int32_t last_count;                        // 上次脉冲数
    int32_t last_count_for_speed;              // 上次速度脉冲数              
    int16_t speed_rpm;                         // 当前转速(RPM)
    int16_t speed_buffer[ENCODER_FILTER_SIZE]; // 速度滤波缓冲区
    uint8_t buffer_index;                      // 缓冲区索引
    unsigned long last_update_time;            // 上次更新时间（ms）
    float position_mm;                         // 累计位置(mm)
    float last_position_mm;                    // 上次位置(mm)
    uint8_t initialized;                       // 初始化标志
    TIM_HandleTypeDef* htim;                   // 定时器句柄
} Encoder_Data_t;

// 电机操作结果枚举
typedef enum {
    MOTOR_OK = 1,              // 操作成功 
    MOTOR_HARDWARE_ERROR = 0   // 硬件错误 
} Motor_Result_t;

// 编码器句柄类型
typedef uint8_t encoder_handle_t;

// =============================================================================
// 函数声明
// =============================================================================

// 编码器管理函数
encoder_handle_t Encoder_Create(TIM_HandleTypeDef* htim);  // 创建编码器实例
Motor_Result_t Encoder_Init(encoder_handle_t handle);      // 编码器初始化
void Encoder_Destroy(encoder_handle_t handle);             // 销毁编码器实例

// 编码器数据获取函数
int32_t Encoder_GetCount(encoder_handle_t handle);         // 获取编码器累计计数值
int16_t Encoder_GetSpeed(encoder_handle_t handle);         // 获取编码器转速（RPM）
float Encoder_GetPosition(encoder_handle_t handle);        // 获取编码器累计位置（mm）
float Encoder_GetDistance(encoder_handle_t handle);        // 获取编码器累计距离（mm）

// 编码器控制函数
void Encoder_Reset(encoder_handle_t handle);               // 编码器复位
Motor_Result_t Encoder_Calibrate(encoder_handle_t handle); // 编码器校准

// 便捷函数（用于简化常见使用场景）
Motor_Result_t Encoder_QuickInit(TIM_HandleTypeDef* htim, encoder_handle_t* handle); // 快速初始化
uint8_t Encoder_IsValid(encoder_handle_t handle);          // 检查句柄是否有效

#endif