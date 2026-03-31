/**
 * @file config.h
 * @brief 项目配置文件
 * @details 包含引脚定义、参数配置等
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
// ==================== 板级选择 ====================
#if defined(BOARD_ESP32S3)
    #define BOARD_NAME "ESP32-S3-N16R8"
    // ESP32-S3 特定引脚定义
    #define LED_PIN 48  // 内置LED (根据实际板子调整)
    // I2C 引脚
    #define I2C_SDA_PIN     9
    #define I2C_SCL_PIN     8
    // IMU 中断引脚
    #define IMU_INT_PIN     7

    // ==================== 电机引脚配置 ====================
    // 左轮电机
    #define MOTOR_LEFT_PWM_PIN      1      // 左电机PWM
    #define MOTOR_LEFT_DIR_PIN      2      // 左电机方向
    #define MOTOR_LEFT_FG_PIN       3      // 左电机霍尔反馈
    // 右轮电机
    #define MOTOR_RIGHT_PWM_PIN     4      // 右电机PWM
    #define MOTOR_RIGHT_DIR_PIN     5      // 右电机方向
    #define MOTOR_RIGHT_FG_PIN      6      // 右电机霍尔反馈

    // ==================== PWM配置 ====================
    #define MOTOR_PWM_FREQ          20000   // PWM频率 20kHz
    #define MOTOR_PWM_RESOLUTION    8       // PWM分辨率 8位 (0-255)
    #define MOTOR_PWM_CHANNEL_LEFT  0       // 左电机PWM通道
    #define MOTOR_PWM_CHANNEL_RIGHT 1       // 右电机PWM通道

    // ==================== 舵机引脚配置 ====================
    #define SERVO_COUNT             4
    #define SERVO_0_PIN             13
    #define SERVO_1_PIN             14
    #define SERVO_2_PIN             18
    #define SERVO_3_PIN             23

    // 舵机PWM通道（避免与电机0/1通道冲突）
    #define SERVO_PWM_CHANNEL_0     8
    #define SERVO_PWM_CHANNEL_1     9
    #define SERVO_PWM_CHANNEL_2     10
    #define SERVO_PWM_CHANNEL_3     11

    // ==================== 电机参数 ====================
    #define MOTOR_FG_PULSES_PER_REV 6       // 每圈霍尔脉冲数

    // ==================== 平衡控制参数 ====================
    // 默认PID参数（可通过setBalancePID调整）
    #define DEFAULT_BALANCE_KP      15.0f   // 比例系数
    #define DEFAULT_BALANCE_KI      0.0f    // 积分系数（暂不使用）
    #define DEFAULT_BALANCE_KD      0.0f    // 微分系数（暂不使用）

    // 平衡角度限制
    #define BALANCE_ANGLE_LIMIT     45.0f   // 超过此角度停止控制（度）
    #define BALANCE_TARGET_ANGLE    0.0f    // 目标平衡角度（度）

    // 输出限制
    #define BALANCE_OUTPUT_MAX      255     // PWM最大值
    #define BALANCE_OUTPUT_MIN      0       // PWM最小值
    #define BALANCE_DEADZONE        5       // 死区（PWM值小于此值时不输出）

    
#elif defined(BOARD_ESP32WROOM)
    #define BOARD_NAME "ESP32-WROOM-32D"
    // ESP32-WROOM-32D 特定引脚定义
    #define LED_PIN 2   // 内置LED
    // I2C 引脚
    #define I2C_SDA_PIN     21
    #define I2C_SCL_PIN     22
    // IMU 中断引脚
    #define IMU_INT_PIN     4

        // ==================== 电机引脚配置 ====================
    // 左轮电机
    #define MOTOR_LEFT_PWM_PIN      26      // 左电机PWM
    #define MOTOR_LEFT_DIR_PIN      33      // 左电机方向
    #define MOTOR_LEFT_FG_PIN       35      // 左电机霍尔反馈
    // 右轮电机
    #define MOTOR_RIGHT_PWM_PIN     25      // 右电机PWM
    #define MOTOR_RIGHT_DIR_PIN     32      // 右电机方向
    #define MOTOR_RIGHT_FG_PIN      34      // 右电机霍尔反馈

    // ==================== PWM配置 ====================
    #define MOTOR_PWM_FREQ          20000   // PWM频率 20kHz
    #define MOTOR_PWM_RESOLUTION    8       // PWM分辨率 8位 (0-255)
    #define MOTOR_PWM_CHANNEL_LEFT  0       // 左电机PWM通道
    #define MOTOR_PWM_CHANNEL_RIGHT 1       // 右电机PWM通道

    // ==================== 舵机引脚配置 ====================
    #define SERVO_COUNT             4
    #define SERVO_0_PIN             13
    #define SERVO_1_PIN             14
    #define SERVO_2_PIN             18
    #define SERVO_3_PIN             23

    // 舵机PWM通道（避免与电机0/1通道冲突）
    #define SERVO_PWM_CHANNEL_0     8
    #define SERVO_PWM_CHANNEL_1     9
    #define SERVO_PWM_CHANNEL_2     10
    #define SERVO_PWM_CHANNEL_3     11


    // ==================== 电机参数 ====================
    #define MOTOR_FG_PULSES_PER_REV 6       // 每圈霍尔脉冲数

    // ==================== 平衡控制参数 ====================
    // 默认PID参数（可通过setBalancePID调整）
    #define DEFAULT_BALANCE_KP      15.0f   // 比例系数
    #define DEFAULT_BALANCE_KI      0.0f    // 积分系数（暂不使用）
    #define DEFAULT_BALANCE_KD      0.0f    // 微分系数（暂不使用）

    // 平衡角度限制
    #define BALANCE_ANGLE_LIMIT     45.0f   // 超过此角度停止控制（度）
    #define BALANCE_TARGET_ANGLE    0.0f    // 目标平衡角度（度）

    // 输出限制
    #define BALANCE_OUTPUT_MAX      255     // PWM最大值
    #define BALANCE_OUTPUT_MIN      0       // PWM最小值
    #define BALANCE_DEADZONE        5       // 死区（PWM值小于此值时不输出）

    
#else
    #error "请在platformio.ini中定义板级类型"
#endif

// ==================== 通用配置 ====================
#define SERIAL_BAUD_RATE 115200

// ==================== 舵机通用配置 ====================
#define SERVO_PWM_FREQ_HZ        50      // 180°模拟舵机标准50Hz
#define SERVO_PWM_RESOLUTION     16      // 使用16位分辨率提升脉宽控制精度
#define SERVO_MIN_PULSE_US       500     // 0°脉宽
#define SERVO_MAX_PULSE_US       2500    // 180°脉宽
// #define SERVO_INIT_ANGLE_1_DEG   85      // 舵机2上电初始化角度
// #define SERVO_INIT_ANGLE_2_DEG   110      // 舵机1上电初始化角度
// #define SERVO_INIT_ANGLE_3_DEG   105      // 舵机3上电初始化角度
// #define SERVO_INIT_ANGLE_4_DEG   70      // 舵机4上电初始化角度
#define SERVO_INIT_ANGLE_1_DEG   90      // 舵机2上电初始化角度
#define SERVO_INIT_ANGLE_2_DEG   90      // 舵机1上电初始化角度
#define SERVO_INIT_ANGLE_3_DEG   90      // 舵机3上电初始化角度
#define SERVO_INIT_ANGLE_4_DEG   90      // 舵机4上电初始化角度

// ==================== 电机配置 ====================
// TODO: 根据实际硬件配置

// ==================== 传感器配置 ====================
// MPU6050 配置
#define IMU_UPDATE_RATE_HZ  200     // IMU更新频率
#define IMU_USE_DMP         1       // 使用DMP

// 陀螺仪量程选择 (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
#define IMU_GYRO_FS_SEL     3       // ±2000°/s 适合快速运动
// 加速度计量程选择 (0=±2g, 1=±4g, 2=±8g, 3=±16g)
#define IMU_ACCEL_FS_SEL    2       // ±8g 平衡精度和范围
// 数字低通滤波器 (0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz)
#define IMU_DLPF_MODE       3       // 44Hz 滤波

#endif // CONFIG_H
