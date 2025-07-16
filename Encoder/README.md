# 编码器模块

## 模块简介

本模块提供MG370霍尔编码器的完整接口封装，适用于电机控制、机器人定位等需要精确测速和位置检测的应用场景。

**主要特性：**
- 支持多编码器实例管理（最大4个）
- 自动处理定时器溢出
- 实时转速和位置计算  
- 可配置的机械参数
- 硬件无关设计，易于移植

## API函数接口

### 1. 初始化函数

| 函数 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `Encoder_QuickInit()` | 快速初始化编码器 | `TIM_HandleTypeDef *htim`<br>`encoder_handle_t *handle` | `MOTOR_OK`: 成功<br>`MOTOR_HARDWARE_ERROR`: 失败 |
| `Encoder_Create()` | 创建编码器实例 | `TIM_HandleTypeDef *htim` | 编码器句柄 |
| `Encoder_Init()` | 初始化编码器 | `encoder_handle_t handle` | `MOTOR_OK`: 成功<br>`MOTOR_HARDWARE_ERROR`: 失败 |

### 2. 数据读取函数

| 函数 | 功能 | 参数 | 返回值 | 单位 |
|------|------|------|--------|------|
| `Encoder_GetCount()` | 获取累计脉冲数 | `encoder_handle_t handle` | `int32_t` | 脉冲 |
| `Encoder_GetSpeed()` | 获取转速 | `encoder_handle_t handle` | `int16_t` | RPM |
| `Encoder_GetPosition()` | 获取累计位置 | `encoder_handle_t handle` | `float` | mm |
| `Encoder_GetDistance()` | 获取增量距离 | `encoder_handle_t handle` | `float` | mm |

### 3. 控制函数

| 函数 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `Encoder_Reset()` | 复位编码器 | `encoder_handle_t handle` | 无 |
| `Encoder_Calibrate()` | 校准编码器 | `encoder_handle_t handle` | `MOTOR_OK`: 成功<br>`MOTOR_HARDWARE_ERROR`: 失败 |
| `Encoder_Destroy()` | 销毁编码器 | `encoder_handle_t handle` | 无 |
| `Encoder_IsValid()` | 检查句柄有效性 | `encoder_handle_t handle` | 1: 有效<br>0: 无效 |

### 4. 配置参数

在 `encoder_module.h` 中修改以下宏定义适配不同编码器：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ENCODER_PPR` | 13 | 编码器每圈脉冲数 |
| `WHEEL_RADIUS` | 2.4f | 轮胎半径(mm) |
| `GEAR_RATIO` | 34 | 传动比 |
| `ENCODER_SAMPLE_TIME` | 20 | 采样周期(ms) |
| `MAX_ENCODER_COUNT` | 4 | 最大编码器数量 |

## 使用示例

### 1. STM32定时器配置
使用STM32CubeMX配置定时器为编码器模式：
- 模式：Combined Channels → Encoder Mode
- 编码器模式：TI1 and TI2
- 计数周期：65535

### 2. 基本使用
```c
#include "encoder_module.h"

int main(void)
{
    encoder_handle_t encoder;
    
    // 硬件初始化
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();  // 定时器初始化
    
    // 编码器初始化
    if (Encoder_QuickInit(&htim2, &encoder) != MOTOR_OK) {
        Error_Handler();
    }
    
    while(1) {
        // 读取编码器数据
        int32_t count = Encoder_GetCount(encoder);
        int16_t speed = Encoder_GetSpeed(encoder);
        float position = Encoder_GetPosition(encoder);
        
        printf("计数: %ld, 转速: %d RPM, 位置: %.1f mm\n", 
               count, speed, position);
        
        HAL_Delay(100);
    }
}
3. 双编码器差速控制
c// 左右轮编码器
encoder_handle_t left_encoder, right_encoder;

void dual_encoder_init(void)
{
    // 初始化两个编码器
    Encoder_QuickInit(&htim2, &left_encoder);
    Encoder_QuickInit(&htim3, &right_encoder);
}

void differential_control(void)
{
    // 读取左右轮速度
    int16_t left_speed = Encoder_GetSpeed(left_encoder);
    int16_t right_speed = Encoder_GetSpeed(right_encoder);
    
    // 计算车体速度和转向
    int16_t linear_speed = (left_speed + right_speed) / 2;
    int16_t angular_speed = (right_speed - left_speed);
    
    printf("线速度: %d RPM, 角速度: %d\n", linear_speed, angular_speed);
}
4. 位置闭环控制
c// 目标位置控制
void position_control(encoder_handle_t encoder, float target_position)
{
    float current_position = Encoder_GetPosition(encoder);
    float error = target_position - current_position;
    
    if (fabs(error) > 1.0) {  // 死区1mm
        // PID控制或其他控制算法
        int16_t control_output = calculate_pid(error);
        motor_set_pwm(control_output);
    } else {
        motor_stop();
        printf("到达目标位置: %.1f mm\n", current_position);
    }
}

// 定点移动
void move_to_distance(encoder_handle_t encoder, float distance)
{
    Encoder_Reset(encoder);  // 复位当前位置
    
    while(1) {
        float current_position = Encoder_GetPosition(encoder);
        
        if (fabs(current_position) >= fabs(distance)) {
            motor_stop();
            break;
        }
        
        // 根据距离调整速度
        int16_t speed = (distance > 0) ? 50 : -50;
        motor_set_pwm(speed);
        
        HAL_Delay(10);
    }
}
5. 速度平滑滤波
c#define FILTER_SIZE 5

// 速度滤波获取
int16_t get_filtered_speed(encoder_handle_t encoder)
{
    static int16_t speed_buffer[FILTER_SIZE] = {0};
    static uint8_t index = 0;
    int32_t sum = 0;
    
    // 更新缓冲区
    speed_buffer[index] = Encoder_GetSpeed(encoder);
    index = (index + 1) % FILTER_SIZE;
    
    // 计算平均值
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += speed_buffer[i];
    }
    
    return sum / FILTER_SIZE;
}
