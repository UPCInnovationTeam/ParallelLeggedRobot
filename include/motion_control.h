/**
 * @file motion_control.h
 * @brief 轮腿机器人运动控制模块
 * @details 提供自平衡控制、电机驱动等功能
 * 
 * 硬件配置：
 *   - 无刷电机使用PWM控制转速
 *   - DIR引脚控制方向（悬空/高电平=CCW，接地/低电平=CW）
 *   - FG信号线输出霍尔信号，一圈6个脉冲
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <Arduino.h>
#include "config.h"
#include "imu.h"

bool getIMUDataFromTask(IMU_Data_t* data);

// ==================== 电机方向枚举 ====================
typedef enum {
    MOTOR_DIR_CW = 0,       // 顺时针 (DIR接地)
    MOTOR_DIR_CCW = 1       // 逆时针 (DIR悬空/高电平)
} MotorDirection_t;

// ==================== 电机ID枚举 ====================
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorID_t;

// ==================== 运动控制状态 ====================
typedef enum {
    MOTION_OK = 0,
    MOTION_ERROR_IMU,
    MOTION_ERROR_MOTOR,
    MOTION_ANGLE_OVERFLOW,
    MOTION_NOT_INITIALIZED
} MotionStatus_t;

// ==================== 电机状态结构 ====================
typedef struct {
    int16_t pwm;                // 当前PWM值 (-255 ~ 255，负值表示反向)
    MotorDirection_t dir;       // 当前方向
    volatile uint32_t fgCount;  // 霍尔脉冲计数（测频法）
    float rpm;                  // 转速 (RPM)
    uint32_t lastFgTime;        // 上次测速时间（测频法）
    volatile uint32_t lastPulseUs;   // 上次脉冲时刻 (微秒，测周法)
    volatile uint32_t pulseIntervalUs; // 最近一次脉冲间隔 (微秒，测周法)
} MotorState_t;

// ==================== 平衡控制参数结构 ====================
typedef struct {
    float kp;                   // 比例系数
    float ki;                   // 积分系数
    float kd;                   // 微分系数
    float targetAngle;          // 目标角度
    float integralSum;          // 积分累加
    float lastError;            // 上次误差（用于微分）
} BalanceParams_t;

// ==================== 运动控制类 ====================
class MotionControl {
public:
    /**
     * @brief 构造函数
     * @param imuPtr IMU实例指针
     */
    MotionControl(IMU* imuPtr);

    /**
     * @brief 初始化运动控制模块
     * @return MotionStatus_t 初始化状态
     */
    MotionStatus_t begin();

    /**
     * @brief 更新平衡控制（需要在主循环中定期调用）
     * @return MotionStatus_t 控制状态
     * @details 使用轮询模式获取IMU数据，计算恢复力矩并输出到电机
     *          公式：恢复力矩 = kp * 俯仰角
     */
    MotionStatus_t updateBalance();

    /**
     * @brief 设置平衡PID参数
     * @param kp 比例系数
     * @param ki 积分系数
     * @param kd 微分系数
     */
    void setBalancePID(float kp, float ki = 0.0f, float kd = 0.0f);

    /**
     * @brief 设置目标平衡角度
     * @param angle 目标角度（度）
     */
    void setTargetAngle(float angle);

    /**
     * @brief 设置单个电机速度
     * @param motor 电机ID
     * @param speed 速度 (-255 ~ 255，负值表示反向)
     */
    void setMotorSpeed(MotorID_t motor, int16_t speed);

    /**
     * @brief 设置两个电机速度
     * @param leftSpeed 左电机速度
     * @param rightSpeed 右电机速度
     */
    void setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * @brief 停止所有电机
     */
    void stopAllMotors();

    /**
     * @brief 启用/禁用平衡控制
     * @param enable 是否启用
     */
    void enableBalance(bool enable);

    /**
     * @brief 检查平衡控制是否启用
     */
    bool isBalanceEnabled() const { return balanceEnabled; }

    /**
     * @brief 获取当前俯仰角
     */
    float getCurrentPitch() const { return currentPitch; }

    /**
     * @brief 获取当前控制输出
     */
    float getControlOutput() const { return controlOutput; }

    /**
     * @brief 获取电机状态
     * @param motor 电机ID
     * @return MotorState_t 电机状态
     */
    MotorState_t getMotorState(MotorID_t motor) const;

    /**
     * @brief 获取电机转速（RPM）
     * @param motor 电机ID
     * @return float 转速
     */
    float getMotorRPM(MotorID_t motor);

    /**
     * @brief 打印调试信息
     */
    void printDebugInfo();

private:
    IMU* imu;                           // IMU实例指针
    bool initialized;                    // 初始化标志
    bool balanceEnabled;                 // 平衡控制启用标志
    IMU_Data_t imuData;                    // 最新的IMU数据

    // 平衡控制参数
    BalanceParams_t balanceParams;
    float currentPitch;                  // 当前俯仰角
    float controlOutput;                 // 控制输出

    // 电机状态
    MotorState_t motorLeft;
    MotorState_t motorRight;

    // 私有方法
    void initMotorPins();                // 初始化电机引脚
    void initPWM();                      // 初始化PWM
    void initFGInterrupts();             // 初始化霍尔信号中断

    /**
     * @brief 计算平衡控制输出
     * @param pitch 当前俯仰角
     * @return float 控制输出 (-255 ~ 255)
     */
    float calculateBalanceOutput(float pitch);

    /**
     * @brief 应用电机输出
     * @param output 控制输出值
     */
    void applyMotorOutput(float output);

    /**
     * @brief 设置单个电机PWM和方向
     */
    void setMotorPWM(MotorID_t motor, uint8_t pwm, MotorDirection_t dir);

    // 霍尔信号中断处理
    static void IRAM_ATTR onLeftFGInterrupt();
    static void IRAM_ATTR onRightFGInterrupt();
    static MotionControl* instance;      // 用于静态中断处理
};

// ==================== 全局实例声明 ====================
extern MotionControl* motionControl;

#endif // MOTION_CONTROL_H
