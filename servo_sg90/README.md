# 舵机控制模块

## 模块简介

本模块实现了通用舵机角度控制功能，支持0-180°精确定位和速度控制，适用于各类PWM控制的舵机。

**主要特性：**
- 角度精确控制（0-180°）
- 可调速度控制
- 支持多路舵机（最多8路）
- 硬件无关设计，易于移植

## API函数接口

### 1. 初始化函数
```c
void servo_init(void);
```
**说明：** 初始化舵机模块，设置所有舵机到90°中位

### 2. 角度控制函数
```c
int8_t servo_set_angle(uint8_t id, float angle);
```
**参数：**
- `id`: 舵机ID（0-7）
- `angle`: 目标角度（0.0-180.0）

**说明：** 立即转动到指定角度

### 3. 速度控制函数
```c
int8_t servo_set_angle_speed(uint8_t id, float angle, float speed);
```
**参数：**
- `id`: 舵机ID（0-7）
- `angle`: 目标角度（0.0-180.0）
- `speed`: 转动速度（°/s）

**说明：** 以指定速度转动到目标角度

### 4. 状态更新函数
```c
void servo_update(void);
```
**说明：** 更新舵机状态，需在主循环中定期调用（建议10ms）

### 5. 用户实现函数（需要前往.c文件中实现）
```c
// PWM输出函数
void user_pwm_set_pulse(uint8_t channel, uint16_t pulse);

// 系统时间函数
uint32_t user_get_tick(void);
```

## 使用示例

### 1. 初始化配置
```c
int main(void)
{
    // 硬件初始化
    HAL_Init();
    MX_TIM3_Init();  // 初始化PWM定时器
    
    // 舵机模块初始化
    servo_init();
    
    while(1) {
        // 更新舵机状态
        servo_update();
        HAL_Delay(10);  // 10ms更新周期
    }
}
```

### 2. 实现PWM输出
```c
// STM32 HAL库示例
void user_pwm_set_pulse(uint8_t channel, uint16_t pulse)
{
    switch(channel) {
        case 0:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
            break;
        case 1:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
            break;
        // 更多通道...
    }
}

// 获取系统时间
uint32_t user_get_tick(void)
{
    return HAL_GetTick();
}
```

### 3. 基本控制示例
```c
// 立即转到指定角度
servo_set_angle(0, 45.0);     // 舵机0转到45°
servo_set_angle(1, 135.0);    // 舵机1转到135°

// 慢速转动
servo_set_angle_speed(0, 180.0, 30.0);  // 以30°/s转到180°

// 快速转动
servo_set_angle_speed(1, 0.0, 120.0);   // 以120°/s转到0°

// 获取当前角度
float angle = servo_get_angle(0);
printf("Servo 0 angle: %.1f\n", angle);

// 停止舵机
servo_stop(0);
```

### 4. 扫描动作示例
```c
// 舵机扫描动作
void servo_scan_demo(void)
{
    // 慢速扫描0-180°
    servo_set_angle_speed(0, 0.0, 45.0);
    while(servo_get_angle(0) > 1.0) {
        servo_update();
        HAL_Delay(10);
    }
    
    servo_set_angle_speed(0, 180.0, 45.0);
    while(servo_get_angle(0) < 179.0) {
        servo_update();
        HAL_Delay(10);
    }
}
```

### 5. 多舵机协调控制
```c
// 机械臂示例
void robot_arm_demo(void)
{
    // 初始位置
    servo_set_angle(0, 90.0);  // 底座
    servo_set_angle(1, 45.0);  // 大臂
    servo_set_angle(2, 135.0); // 小臂
    servo_set_angle(3, 90.0);  // 夹爪
    
    HAL_Delay(1000);
    
    // 同步运动到目标位置
    servo_set_angle_speed(0, 45.0, 60.0);
    servo_set_angle_speed(1, 90.0, 45.0);
    servo_set_angle_speed(2, 90.0, 45.0);
    servo_set_angle_speed(3, 30.0, 90.0);
    
    // 等待运动完成
    while(g_servos[0].is_moving || g_servos[1].is_moving || 
          g_servos[2].is_moving || g_servos[3].is_moving) {
        servo_update();
        HAL_Delay(10);
    }
}
```

## 参数配置

在`servo_control.h`中可修改以下参数：

```c
#define SERVO_PWM_PERIOD    20000   // PWM周期(us)
#define SERVO_MIN_PULSE     500     // 最小脉宽(us)
#define SERVO_MAX_PULSE     2500    // 最大脉宽(us)
#define SERVO_MAX_NUM       8       // 最大舵机数量
```

## 注意事项

1. PWM频率需配置为50Hz（周期20ms）
2. 根据舵机型号调整脉宽范围
3. `servo_update()`调用频率影响速度控制精度
4. 确保电源供电充足（舵机启动电流较大）