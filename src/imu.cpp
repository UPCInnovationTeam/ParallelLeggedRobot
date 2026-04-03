#include "imu.h"
#include <Arduino.h>
#include <math.h>

IMU::IMU()
{
    // 初始化数据
    accelData.ax = 0;
    accelData.ay = 0;
    accelData.az = 0;
    gyroData.gx = 0;
    gyroData.gy = 0;
    gyroData.gz = 0;
    imuData.pitch = 0;
    imuData.roll = 0;
    imuData.yaw = 0;
    imuData.valid = false;
    imuData.timestamp = 0;
    imuData.accelData = &accelData;
    imuData.gyroData = &gyroData;
}

bool IMU::begin()
{
    // 唤醒ICM20602（默认进入睡眠模式）
    Wire.beginTransmission(ICM20602_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0);                        // 写入0唤醒设备
    Wire.endTransmission(true);

    /*加速度寄存器配置*/
    Wire.beginTransmission(ICM20602_ADDR);
    Wire.write(A_ADDR);
    Wire.write(a_full_scale[AFS_SEL]);
    Wire.endTransmission(true);
    factor_a = a_scale[AFS_SEL];

    /*角速度寄存器配置*/
    Wire.beginTransmission(ICM20602_ADDR);
    Wire.write(G_ADDR);
    Wire.write(g_full_scale[GFS_SEL]);
    Wire.endTransmission(true);
    factor_g = g_scale[GFS_SEL];
    return true;
}

// 读取加速度计数据函数
void IMU::readAccelData() {
  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20602_ADDR, 6, true);

  accelData.ax = (Wire.read() << 8) | Wire.read();  // ACCEL_XOUT
  accelData.ay = (Wire.read() << 8) | Wire.read();  // ACCEL_YOUT
  accelData.az = (Wire.read() << 8) | Wire.read();  // ACCEL_ZOUT
}

// 读取陀螺仪数据函数
void IMU::readGyroData() {
  Wire.beginTransmission(ICM20602_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20602_ADDR, 6, true);

  gyroData.gx = (Wire.read() << 8) | Wire.read();  // GYRO_XOUT
  gyroData.gy = (Wire.read() << 8) | Wire.read();  // GYRO_YOUT
  gyroData.gz = (Wire.read() << 8) | Wire.read();  // GYRO_ZOUT
}

void IMU::update()
{
  // 1) 读取原始数据
  readAccelData();
  readGyroData();

  // 2) 计算 dt
  uint32_t nowUs = micros();
  static uint32_t lastUs = 0;
  if (lastUs == 0) {
    lastUs = nowUs;
    imuData.timestamp = millis();
    imuData.valid = false;
    return;
  }
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;

  // 防止异常 dt 影响滤波
  if (dt <= 0.0001f || dt > 0.05f) {
    dt = 0.005f; // 200Hz 默认值
  }

  // 3) 单位转换
  // 你的 factor_g 对应 LSB/(deg/s)，所以除以 factor_g 得到 deg/s
  float gx = (float)gyroData.gx / factor_g;
  float gy = (float)gyroData.gy / factor_g;
  float gz = (float)gyroData.gz / factor_g;

  // 4) 加速度计解算倾角（单位：deg）
  float ax = (float)accelData.ax;
  float ay = (float)accelData.ay;
  float az = (float)accelData.az;

  float rollAcc  = atan2f(ay, az) * 57.29578f;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;

  // 5) 陀螺积分
  float rollGyro  = imuData.roll  + gx * dt;
  float pitchGyro = imuData.pitch + gy * dt;
  float yawGyro   = imuData.yaw   + gz * dt;

  // 6) 互补滤波
  const float alpha = 0.98f;
  imuData.roll  = alpha * rollGyro  + (1.0f - alpha) * rollAcc;
  imuData.pitch = alpha * pitchGyro + (1.0f - alpha) * pitchAcc;

  // 7) yaw 无磁力计只能纯积分，会漂移
  imuData.yaw = yawGyro;

  imuData.timestamp = millis();
  imuData.valid = true;

}
