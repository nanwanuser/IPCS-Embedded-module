# 姿态估计模块

## 模块简介

本模块实现了基于四阶龙格-库塔积分的高精度姿态估计算法，支持6轴和9轴两种工作模式。

**主要特性：**
- 四阶龙格-库塔积分，高精度姿态更新
- 支持6轴（陀螺仪+加速度计）和9轴（+磁力计）模式
- 输出四元数和欧拉角
- 可调节收敛速度参数
- 适用于无人机、机器人、平衡车等应用

## API函数接口

### 1. 初始化函数
```c
AhrsStatus nanwan_ahrs_init(AhrsConfig *config);
```
初始化AHRS模块，config为NULL时使用默认配置

### 2. 姿态更新函数
```c
// 9轴更新（自动检测磁力计）
AhrsStatus nanwan_ahrs_update(AhrsSensorData *sensor);

// 6轴更新
AhrsStatus nanwan_ahrs_update_imu(float gx, float gy, float gz, 
                                   float ax, float ay, float az);
```

### 3. 获取姿态函数
```c
// 获取完整姿态数据
AhrsStatus nanwan_ahrs_get_attitude(AhrsAttitude *attitude);

// 获取四元数
AhrsStatus nanwan_ahrs_get_quaternion(float q[4]);

// 获取欧拉角（弧度）
AhrsStatus nanwan_ahrs_get_euler(float *roll, float *pitch, float *yaw);

// 获取欧拉角（度数）
AhrsStatus nanwan_ahrs_get_euler_degrees(float *roll, float *pitch, float *yaw);
```

### 4. 其他函数
```c
// 重置姿态
AhrsStatus nanwan_ahrs_reset(void);

// 设置收敛速度（0.01-0.5）
AhrsStatus nanwan_ahrs_set_beta(float beta);
```

## 使用示例

### 1. 初始化
```c
int main(void)
{
    // 硬件初始化
    HAL_Init();
    IMU_Init();
    
    // 默认配置初始化（100Hz，beta=0.1）
    nanwan_ahrs_init(NULL);
    
    // 或自定义配置
    AhrsConfig config = {
        .sample_freq = 200.0f,  // 200Hz
        .beta = 0.05f,          // 平滑模式
        .use_mag = 1            // 使用磁力计
    };
    nanwan_ahrs_init(&config);
    
    while(1);
}
```

### 2. 定时更新姿态
```c
// 定时器中断（与sample_freq一致）【Freertos可使用VtaskDelayUntil()实现定时更新】
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        // 9轴更新
        AhrsSensorData sensor;
        IMU_GetGyroData(&sensor.gx, &sensor.gy, &sensor.gz);    // rad/s
        IMU_GetAccelData(&sensor.ax, &sensor.ay, &sensor.az);   // 任意单位
        IMU_GetMagData(&sensor.mx, &sensor.my, &sensor.mz);     // 任意单位
        nanwan_ahrs_update(&sensor);
        
        // 或6轴更新
        // nanwan_ahrs_update_imu(gx, gy, gz, ax, ay, az);
    }
}
```

### 3. 获取姿态
```c
void control_task(void)
{
    float roll_deg, pitch_deg, yaw_deg;
    
    // 直接获取度数值
    nanwan_ahrs_get_euler_degrees(&roll_deg, &pitch_deg, &yaw_deg);
    
    // 用于控制
    printf("姿态: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n", 
           roll_deg, pitch_deg, yaw_deg);
}
```
