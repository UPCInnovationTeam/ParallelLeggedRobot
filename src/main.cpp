/**
 * @file main.cpp
 * @brief 轮腿机器人主程序入口
 */


#include <Arduino.h>
#include "config.h"

// #include "motion_control.h"
// #include "servo_control.h"
#include "imu.h"

IMU imu;

void setup() {
    // 初始化串口
    Serial.begin(SERIAL_BAUD_RATE);

    // 初始化iic
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(1000);
    imu.begin();

    Serial.println("================================");
    Serial.println("轮腿机器人");
    Serial.print("开发板: ");
    Serial.println(BOARD_NAME);
    Serial.println("================================");
    // 打印FreeRTOS信息
    Serial.println("系统信息:");
    Serial.print("  FreeRTOS版本: ");
    Serial.println(tskKERNEL_VERSION_NUMBER);
    Serial.print("  系统时钟: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.print("  Flash大小: ");
    Serial.print(ESP.getFlashChipSize() / (1024 * 1024));
    Serial.println(" MB");
    Serial.println();

    // motionControl = new MotionControl(&imu);

    // // 初始化运动控制系统
    // if (motionControl->begin() != MOTION_OK) {
    //     Serial.println("运动控制系统初始化失败!");
    //     while (1) { delay(100); }
    // }
    // motionControl->enableBalance(false);

    // 初始化所有180°舵机（默认90°中位）
    // if (!servoControl.begin()) {
    //     Serial.println("舵机初始化失败!");
    //     while (1) { delay(100); }
    // }
}
void loop() {
    imu.readAccelData();
    imu.readGyroData();
    Serial.printf("IMU数据: ax=%.2f ay=%.2f az=%.2f | gx=%.2f gy=%.2f gz=%.2f\n",
                  imu.accelData.ax, imu.accelData.ay, imu.accelData.az,
                  imu.gyroData.gx, imu.gyroData.gy, imu.gyroData.gz);

    // 添加小延迟避免过度占用CPU
    delay(100);
}
