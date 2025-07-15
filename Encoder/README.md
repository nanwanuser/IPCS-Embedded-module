# 编码器模块 (Encoder Module)

## 简介

本模块是西南科技大学智能系统与智慧服务创新实践班嵌入式通用模块库的一部分，专为全国大学生电子设计大赛设计。提供完整的MG370霍尔编码器接口封装，支持转速检测、位置计算等功能。

## 特性

- 支持多编码器实例管理（最大4个）
- 自动处理定时器溢出
- 实时转速和位置计算
- 简单易用的API接口
- **支持CMake构建系统（不支持MDK）**

## 工具链要求

⚠️ **重要：本模块使用CMake构建，不支持MDK**

- **构建工具**: CMake 3.16+
- **编译器**: arm-none-eabi-gcc
- **推荐IDE**: CLion / VS Code + CMake Tools

## 硬件配置

### MG370编码器参数
- 每圈脉冲数：13 PPR
- 四倍频：52脉冲/转
- 传动比：34:1
- 轮胎半径：2.4mm（可修改）

### STM32定时器配置
使用STM32CubeMX配置定时器为编码器模式：
1. 选择TIM2/TIM3/TIM4等定时器
2. 模式设置：Combined Channels → Encoder Mode
3. Encoder Mode: TI1 and TI2
4. Counter Period: 65535

## 快速开始

### 1. 集成到项目

将以下文件复制到项目中：
```
encoder_module.h
encoder_module.c
```

### 2. CMake配置

```cmake
# 添加编码器模块
add_library(encoder_module encoder_module.c)
target_include_directories(encoder_module PUBLIC .)

# 链接到主程序
target_link_libraries(your_target encoder_module)
```

### 3. 基本使用

```c
#include "encoder_module.h"

encoder_handle_t encoder;

int main(void) {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    MX_TIM2_Init();  // 定时器初始化
    
    // 编码器初始化
    if (Encoder_QuickInit(&htim2, &encoder) != MOTOR_OK) {
        Error_Handler();
    }
    
    while (1) {
        // 读取数据
        int32_t count = Encoder_GetCount(encoder);
        int16_t speed = Encoder_GetSpeed(encoder);
        float position = Encoder_GetPosition(encoder);
        
        printf("计数: %ld, 转速: %d RPM, 位置: %.1f mm\n", 
               count, speed, position);
        
        HAL_Delay(100);
    }
}
```

## API参考

### 数据类型

| 类型 | 说明 |
|------|------|
| `encoder_handle_t` | 编码器句柄（uint8_t） |
| `Motor_Result_t` | 操作结果（MOTOR_OK/MOTOR_HARDWARE_ERROR） |

### 管理函数

| 函数 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `Encoder_Create()` | 创建编码器实例 | `TIM_HandleTypeDef* htim` | 编码器句柄 |
| `Encoder_Init()` | 初始化编码器 | `encoder_handle_t handle` | 操作结果 |
| `Encoder_QuickInit()` | 快速初始化（推荐） | `TIM_HandleTypeDef* htim, encoder_handle_t* handle` | 操作结果 |
| `Encoder_Destroy()` | 销毁编码器 | `encoder_handle_t handle` | 无 |

### 数据获取函数

| 函数 | 功能 | 参数 | 返回值 | 单位 |
|------|------|------|--------|------|
| `Encoder_GetCount()` | 获取累计脉冲数 | `encoder_handle_t handle` | `int32_t` | 脉冲 |
| `Encoder_GetSpeed()` | 获取转速 | `encoder_handle_t handle` | `int16_t` | RPM |
| `Encoder_GetPosition()` | 获取累计位置 | `encoder_handle_t handle` | `float` | mm |
| `Encoder_GetDistance()` | 获取增量距离 | `encoder_handle_t handle` | `float` | mm |

### 控制函数

| 函数 | 功能 | 参数 | 返回值 |
|------|------|------|--------|
| `Encoder_Reset()` | 复位编码器 | `encoder_handle_t handle` | 无 |
| `Encoder_Calibrate()` | 校准编码器 | `encoder_handle_t handle` | 操作结果 |
| `Encoder_IsValid()` | 检查句柄有效性 | `encoder_handle_t handle` | 1=有效，0=无效 |

## 使用示例

### 单编码器使用

```c
encoder_handle_t my_encoder;

// 初始化
Encoder_QuickInit(&htim2, &my_encoder);

// 读取数据
int32_t count = Encoder_GetCount(my_encoder);
int16_t speed = Encoder_GetSpeed(my_encoder);
float position = Encoder_GetPosition(my_encoder);
```

### 双编码器使用

```c
encoder_handle_t left_encoder, right_encoder;

// 初始化两个编码器
Encoder_QuickInit(&htim2, &left_encoder);
Encoder_QuickInit(&htim3, &right_encoder);

// 读取两轮数据
int16_t left_speed = Encoder_GetSpeed(left_encoder);
int16_t right_speed = Encoder_GetSpeed(right_encoder);
```

### 复位和校准

```c
// 复位编码器
Encoder_Reset(my_encoder);

// 校准编码器
if (Encoder_Calibrate(my_encoder) == MOTOR_OK) {
    printf("校准成功\n");
}
```

## 参数配置

在 `encoder_module.h` 中可修改的主要参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ENCODER_PPR` | 13 | 编码器每圈脉冲数 |
| `WHEEL_RADIUS` | 2.4f | 轮胎半径（mm） |
| `GEAR_RATIO` | 34 | 传动比 |
| `ENCODER_SAMPLE_TIME` | 20 | 转速采样周期（ms） |
| `MAX_ENCODER_COUNT` | 4 | 最大编码器数量 |

## 故障排除

| 问题 | 解决方案 |
|------|----------|
| 编码器计数不变 | 检查定时器是否正确初始化为编码器模式 |
| 速度读数异常 | 检查采样周期设置和编码器连线 |
| 初始化失败 | 确认定时器句柄有效且未被占用 |
| 位置计算错误 | 检查机械参数配置是否正确 |

## 移植说明

### 适配其他编码器
1. 修改 `ENCODER_PPR` 为实际编码器的PPR值
2. 调整 `WHEEL_RADIUS` 和 `GEAR_RATIO` 参数
3. 根据需要修改采样周期

### 适配其他STM32型号
1. 确认定时器支持编码器接口
2. 根据定时器位数调整溢出参数
3. 检查HAL库兼容性

## 贡献指南

1. Fork本仓库
2. 创建新分支 (`git checkout -b feature/your-feature`)
3. 提交更改 (`git commit -m 'Add: 功能描述'`)
4. 推送到分支 (`git push origin feature/your-feature`)
5. 创建Pull Request

## 开源许可证

MIT License