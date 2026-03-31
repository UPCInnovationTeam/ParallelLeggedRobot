/**
 * @file motion_control.cpp
 * @brief 轮腿机器人运动控制模块实现
 */

#include "motion_control.h"

// 静态实例指针（用于中断处理）
MotionControl* MotionControl::instance = nullptr;

// 全局实例
MotionControl* motionControl = nullptr;

// ==================== 构造函数 ====================
MotionControl::MotionControl(IMU* imuPtr) : imu(imuPtr) {
    initialized = false;
    balanceEnabled = false;
    currentPitch = 0.0f;
    controlOutput = 0.0f;

    // 初始化平衡参数
    balanceParams.kp = DEFAULT_BALANCE_KP;
    balanceParams.ki = DEFAULT_BALANCE_KI;
    balanceParams.kd = DEFAULT_BALANCE_KD;
    balanceParams.targetAngle = BALANCE_TARGET_ANGLE;
    balanceParams.integralSum = 0.0f;
    balanceParams.lastError = 0.0f;

    // 初始化电机状态
    memset(&motorLeft, 0, sizeof(MotorState_t));
    memset(&motorRight, 0, sizeof(MotorState_t));

    // 设置静态实例指针
    instance = this;
}

// ==================== 初始化 ====================
MotionStatus_t MotionControl::begin() {
    Serial.println("[MotionControl] 初始化开始...");

    // 检查IMU
    if (imu == nullptr) {
        Serial.println("[MotionControl] 错误: IMU实例为空!");
        return MOTION_ERROR_IMU;
    }

    // 初始化电机引脚
    initMotorPins();

    // 初始化PWM
    initPWM();

    // 初始化霍尔信号中断
    initFGInterrupts();

    // 确保电机停止
    stopAllMotors();

    initialized = true;
    Serial.println("[MotionControl] 初始化完成!");
    Serial.printf("[MotionControl] 平衡PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                  balanceParams.kp, balanceParams.ki, balanceParams.kd);

    return MOTION_OK;
}

// ==================== 初始化电机引脚 ====================
void MotionControl::initMotorPins() {
    Serial.println("[MotionControl] 配置电机引脚...");

    // 左电机
    pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_FG_PIN, INPUT_PULLUP);

    // 右电机
    pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_FG_PIN, INPUT_PULLUP);
    

    // 默认方向
    digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);   // CW
    digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);  // CW

    Serial.printf("[MotionControl] 左电机: PWM=%d, DIR=%d, FG=%d\n",
                  MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_FG_PIN);
    Serial.printf("[MotionControl] 右电机: PWM=%d, DIR=%d, FG=%d\n",
                  MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_FG_PIN);
}

// ==================== 初始化PWM ====================
void MotionControl::initPWM() {
    Serial.println("[MotionControl] 配置PWM...");

    // 配置PWM通道
    ledcSetup(MOTOR_PWM_CHANNEL_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(MOTOR_PWM_CHANNEL_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

    // 绑定PWM通道到引脚
    ledcAttachPin(MOTOR_LEFT_PWM_PIN, MOTOR_PWM_CHANNEL_LEFT);
    ledcAttachPin(MOTOR_RIGHT_PWM_PIN, MOTOR_PWM_CHANNEL_RIGHT);

    // 初始占空比为0
    ledcWrite(MOTOR_PWM_CHANNEL_LEFT, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, 0);

    Serial.printf("[MotionControl] PWM频率: %dHz, 分辨率: %d位\n",
                  MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
}

// ==================== 初始化霍尔信号中断 ====================
void MotionControl::initFGInterrupts() {
    Serial.println("[MotionControl] 配置霍尔信号中断...");

    // 左电机FG中断
    attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_FG_PIN),
                    onLeftFGInterrupt, RISING);

    // 右电机FG中断
    attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_FG_PIN),
                    onRightFGInterrupt, RISING);

    // 初始化计数和时间
    motorLeft.fgCount = 0;
    motorLeft.lastFgTime = millis();
    motorLeft.lastPulseUs = 0;
    motorLeft.pulseIntervalUs = 0;
    motorRight.fgCount = 0;
    motorRight.lastFgTime = millis();
    motorRight.lastPulseUs = 0;
    motorRight.pulseIntervalUs = 0;

    Serial.println("[MotionControl] 霍尔信号中断已启用");
}

// ==================== 霍尔信号中断处理 ====================
void IRAM_ATTR MotionControl::onLeftFGInterrupt() {
    if (instance != nullptr) {
        uint32_t now = micros();
        instance->motorLeft.pulseIntervalUs = now - instance->motorLeft.lastPulseUs;
        instance->motorLeft.lastPulseUs = now;
        instance->motorLeft.fgCount++;
    }
}

void IRAM_ATTR MotionControl::onRightFGInterrupt() {
    if (instance != nullptr) {
        uint32_t now = micros();
        instance->motorRight.pulseIntervalUs = now - instance->motorRight.lastPulseUs;
        instance->motorRight.lastPulseUs = now;
        instance->motorRight.fgCount++;
    }
}

// ==================== 更新平衡控制 ====================
MotionStatus_t MotionControl::updateBalance() {
    if (!initialized) {
        return MOTION_NOT_INITIALIZED;
    }

    // 轮询模式更新IMU数据
    if (!getIMUDataFromTask(&imuData)) {
        return MOTION_ERROR_IMU;
    }

    currentPitch = imuData.pitch;

    // 检查角度是否超限
    if (fabsf(currentPitch) > BALANCE_ANGLE_LIMIT) {
        stopAllMotors();
        return MOTION_ANGLE_OVERFLOW;
    }

    // 如果平衡控制未启用，直接返回
    if (!balanceEnabled) {
        return MOTION_OK;
    }

    // 计算控制输出
    controlOutput = calculateBalanceOutput(currentPitch);

    // 应用到电机
    applyMotorOutput(controlOutput);

    return MOTION_OK;
}

// ==================== 计算平衡控制输出 ====================
float MotionControl::calculateBalanceOutput(float pitch) {
    // 计算误差：目标角度 - 当前角度
    float error = balanceParams.targetAngle - pitch;

    // P控制：恢复力矩 = Kp * 俯仰角偏差
    // 注意：这里使用的是角度偏差，正角度（向前倾）需要向后的恢复力
    float pTerm = balanceParams.kp * error;

    // I控制（可选）
    float iTerm = 0.0f;
    if (balanceParams.ki != 0.0f) {
        balanceParams.integralSum += error;
        // 积分限幅，防止积分饱和
        balanceParams.integralSum = constrain(balanceParams.integralSum, -100.0f, 100.0f);
        iTerm = balanceParams.ki * balanceParams.integralSum;
    }

    // D控制（可选）
    float dTerm = 0.0f;
    if (balanceParams.kd != 0.0f) {
        dTerm = balanceParams.kd * (error - balanceParams.lastError);
        balanceParams.lastError = error;
    }

    // 计算总输出
    float output = pTerm + iTerm + dTerm;

    // 输出限幅
    output = constrain(output, -BALANCE_OUTPUT_MAX, BALANCE_OUTPUT_MAX);

    return output;
}

// ==================== 应用电机输出 ====================
void MotionControl::applyMotorOutput(float output) {
    // 死区处理
    if (fabsf(output) < BALANCE_DEADZONE) {
        setMotorPWM(MOTOR_LEFT, 0, MOTOR_DIR_CW);
        setMotorPWM(MOTOR_RIGHT, 0, MOTOR_DIR_CW);
        return;
    }

    // 确定方向和PWM值
    MotorDirection_t dir;
    uint8_t pwm;

    if (output >= 0) {
        // 正输出：向前（假设CCW为前进方向，根据实际接线调整）
        dir = MOTOR_DIR_CCW;
        pwm = (uint8_t)fabsf(output);
    } else {
        // 负输出：向后
        dir = MOTOR_DIR_CW;
        pwm = (uint8_t)fabsf(output);
    }

    // 左右电机同步输出（用于平衡）
    // 注意：如果左右电机安装方向相反，需要调整其中一个的方向
    setMotorPWM(MOTOR_LEFT, pwm, dir);
    setMotorPWM(MOTOR_RIGHT, pwm, dir);  // 如果需要镜像，改为相反方向
}

// ==================== 设置电机PWM和方向 ====================
void MotionControl::setMotorPWM(MotorID_t motor, uint8_t pwm, MotorDirection_t dir) {
    if (motor == MOTOR_LEFT) {
        // 设置方向：LOW=CW, HIGH=CCW（悬空时为高电平）
        digitalWrite(MOTOR_LEFT_DIR_PIN, dir == MOTOR_DIR_CCW ? HIGH : LOW);
        // 设置PWM
        ledcWrite(MOTOR_PWM_CHANNEL_LEFT, pwm);
        // 更新状态
        motorLeft.pwm = (dir == MOTOR_DIR_CCW) ? pwm : -pwm;
        motorLeft.dir = dir;
    } else {
        digitalWrite(MOTOR_RIGHT_DIR_PIN, dir == MOTOR_DIR_CCW ? HIGH : LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, pwm);
        motorRight.pwm = (dir == MOTOR_DIR_CCW) ? pwm : -pwm;
        motorRight.dir = dir;
    }
}

// ==================== 设置平衡PID参数 ====================
void MotionControl::setBalancePID(float kp, float ki, float kd) {
    balanceParams.kp = kp;
    balanceParams.ki = ki;
    balanceParams.kd = kd;
    // 重置积分项
    balanceParams.integralSum = 0.0f;
    balanceParams.lastError = 0.0f;

    Serial.printf("[MotionControl] PID更新: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
}

// ==================== 设置目标角度 ====================
void MotionControl::setTargetAngle(float angle) {
    balanceParams.targetAngle = angle;
    // 重置积分项
    balanceParams.integralSum = 0.0f;
    Serial.printf("[MotionControl] 目标角度: %.2f°\n", angle);
}

// ==================== 设置单个电机速度 ====================
void MotionControl::setMotorSpeed(MotorID_t motor, int16_t speed) {
    MotorDirection_t dir = (speed >= 0) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
    uint8_t pwm = (uint8_t)min(abs(speed), 255);
    setMotorPWM(motor, pwm, dir);
}

// ==================== 设置两个电机速度 ====================
void MotionControl::setMotorSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    setMotorSpeed(MOTOR_LEFT, leftSpeed);
    setMotorSpeed(MOTOR_RIGHT, rightSpeed);
}

// ==================== 停止所有电机 ====================
void MotionControl::stopAllMotors() {
    setMotorPWM(MOTOR_LEFT, 0, MOTOR_DIR_CW);
    setMotorPWM(MOTOR_RIGHT, 0, MOTOR_DIR_CW);

    // 重置积分项
    balanceParams.integralSum = 0.0f;
}

// ==================== 启用/禁用平衡控制 ====================
void MotionControl::enableBalance(bool enable) {
    if (enable && !balanceEnabled) {
        // 启用时重置积分
        balanceParams.integralSum = 0.0f;
        balanceParams.lastError = 0.0f;
        Serial.println("[MotionControl] 平衡控制已启用");
    } else if (!enable && balanceEnabled) {
        stopAllMotors();
        Serial.println("[MotionControl] 平衡控制已禁用");
    }
    balanceEnabled = enable;
}

// ==================== 获取电机状态 ====================
MotorState_t MotionControl::getMotorState(MotorID_t motor) const {
    return (motor == MOTOR_LEFT) ? motorLeft : motorRight;
}

// ==================== 获取电机转速 ====================
float MotionControl::getMotorRPM(MotorID_t motor) {
    MotorState_t* state = (motor == MOTOR_LEFT) ? &motorLeft : &motorRight;

    // 满转速120RPM，每圈6脉冲 => 最小脉冲间隔约 83ms
    // 低速切换阈值：当200ms内脉冲数 < 3 时使用测周法
    // 超时阈值：超过1秒无脉冲认为已停转
    const uint32_t SAMPLE_PERIOD_MS = 200;   // 测频法采样周期
    const uint32_t TIMEOUT_US = 1000000;     // 1秒无脉冲判定停转
    const uint32_t LOW_SPEED_PULSE_THRESHOLD = 3; // 低速脉冲阈值

    uint32_t currentTime = millis();
    uint32_t deltaTime = currentTime - state->lastFgTime;

    if (deltaTime >= SAMPLE_PERIOD_MS) {
        noInterrupts();
        uint32_t pulses = state->fgCount;
        state->fgCount = 0;
        uint32_t interval = state->pulseIntervalUs;
        uint32_t lastPulse = state->lastPulseUs;
        interrupts();

        // 检查是否超时（长时间无脉冲 = 停转）
        uint32_t elapsed = micros() - lastPulse;
        if (lastPulse == 0 || elapsed > TIMEOUT_US) {
            state->rpm = 0.0f;
        } else if (pulses >= LOW_SPEED_PULSE_THRESHOLD) {
            // 高速：测频法 —— 脉冲足够多时精度高
            // RPM = 脉冲数 * 60000 / (每圈脉冲数 * 时间ms)
            state->rpm = (float)pulses * 60000.0f / (MOTOR_FG_PULSES_PER_REV * deltaTime);
        } else if (interval > 0) {
            // 低速：测周法 —— 用最近一次脉冲间隔计算
            // RPM = 60 * 1000000 / (每圈脉冲数 * 脉冲间隔us)
            state->rpm = 60000000.0f / (MOTOR_FG_PULSES_PER_REV * (float)interval);
        } else {
            state->rpm = 0.0f;
        }

        state->lastFgTime = currentTime;
    }

    return state->rpm;
}

// ==================== 打印调试信息 ====================
void MotionControl::printDebugInfo() {
    Serial.println("========== MotionControl Debug ==========");
    Serial.printf("初始化: %s, 平衡启用: %s\n",
                  initialized ? "是" : "否",
                  balanceEnabled ? "是" : "否");
    Serial.printf("俯仰角: %.2f°, 目标: %.2f°\n",
                  currentPitch, balanceParams.targetAngle);
    Serial.printf("控制输出: %.2f\n", controlOutput);
    Serial.printf("PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n",
                  balanceParams.kp, balanceParams.ki, balanceParams.kd);
    Serial.printf("左电机: PWM=%d, 方向=%s, RPM=%.1f\n",
                  motorLeft.pwm,
                  motorLeft.dir == MOTOR_DIR_CCW ? "CCW" : "CW",
                  getMotorRPM(MOTOR_LEFT));
    Serial.printf("右电机: PWM=%d, 方向=%s, RPM=%.1f\n",
                  motorRight.pwm,
                  motorRight.dir == MOTOR_DIR_CCW ? "CCW" : "CW",
                  getMotorRPM(MOTOR_RIGHT));
    Serial.println("==========================================");
}
