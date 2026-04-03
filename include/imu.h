#ifndef IMU_H
#define IMU_H

#define ICM20602_ADDR 0x69  // IIC地址
#define PWR_MGMT_1 0x6B // 电源管理寄存器
#define GYRO_XOUT_H 0x43 // 陀螺仪数据寄存器
#define ACCEL_XOUT_H 0x3B // 加速度计数据寄存器

#include <Wire.h>
#include <Arduino.h>

//加速度配置
#define A_ADDR 0x1C
#define AFS_SEL 1
//陀螺仪配置
#define G_ADDR 0x1B
#define GFS_SEL 2

typedef struct {
    int16_t ax;
    int16_t ay;
    int16_t az;
} AccelData_t;

typedef struct {
    int16_t gx;
    int16_t gy;
    int16_t gz;
} GyroData_t;

// IMU数据结构，包含处理后的欧拉角
typedef struct {
    float pitch;      // 俯仰角 (度)
    float roll;       // 横滚角 (度)
    float yaw;        // 偏航角 (度)
    bool valid;       // 数据有效性标志
    GyroData_t* gyroData;
    AccelData_t* accelData;
    uint32_t timestamp; // 时间戳 (毫秒)
} IMU_Data_t;


class IMU {
    public:
        IMU();
        bool begin();

        // 读取原始数据
        void readAccelData();
        void readGyroData();

        void update();
        AccelData_t accelData;
        GyroData_t gyroData;
        IMU_Data_t imuData;

    private:
        const int g_full_scale[4] = {0x00, 0x08, 0x10, 0x18}; // 陀螺仪满量程选择 0: +/- 250°/s 1: +/- 500°/s 2: +/- 1000°/s 3: +/- 2000°/s
        const int a_full_scale[4] = {0x00, 0x08, 0x10, 0x18}; // 加速度计满量程选择 0: +/- 2g 1: +/- 4g 2: +/- 8g 3: +/- 16g
        const float g_scale[4] = {131, 65.5, 32.8, 16.4};
        const int a_scale[4] = {16384, 8192, 4096, 2048};
        const float GRAVITY = 9.81f; // 重力加速度 m/s²

        float factor_g;
        int factor_a;


};

#endif