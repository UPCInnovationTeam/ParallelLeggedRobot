/**
 * @file servo_control.cpp
 * @brief 180°舵机PWM控制模块实现
 */

#include "servo_control.h"

ServoControl servoControl;

uint8_t ServoControl::servoPins[SERVO_COUNT] = {
    SERVO_0_PIN,
    SERVO_1_PIN,
    SERVO_2_PIN,
    SERVO_3_PIN
};

uint8_t ServoControl::servoChannels[SERVO_COUNT] = {
    SERVO_PWM_CHANNEL_0,
    SERVO_PWM_CHANNEL_1,
    SERVO_PWM_CHANNEL_2,
    SERVO_PWM_CHANNEL_3
};

static const float kServoInitAngles[SERVO_COUNT] = {
    SERVO_INIT_ANGLE_1_DEG,
    SERVO_INIT_ANGLE_2_DEG,
    SERVO_INIT_ANGLE_3_DEG,
    SERVO_INIT_ANGLE_4_DEG
};

ServoControl::ServoControl() : initialized(false) {
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        currentAngles[i] = constrain(kServoInitAngles[i], 0.0f, 180.0f);
    }
}

bool ServoControl::begin() {
    if (initialized) {
        return true;
    }

    Serial.println("[ServoControl] 初始化180°舵机...");

    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        const float initAngle = constrain(kServoInitAngles[i], 0.0f, 180.0f);
        const uint32_t initDuty = angleToDuty(initAngle);
        ledcSetup(servoChannels[i], SERVO_PWM_FREQ_HZ, SERVO_PWM_RESOLUTION);
        // 先预装载0°占空比，再绑定引脚，减少上电瞬间跳变
        ledcWrite(servoChannels[i], initDuty);
        ledcAttachPin(servoPins[i], servoChannels[i]);
        ledcWrite(servoChannels[i], initDuty);
        currentAngles[i] = initAngle;

        Serial.printf("[ServoControl] 舵机%u: PIN=%u CH=%u INIT=%.1f°\n",
                  (unsigned int)(i + 1), servoPins[i], servoChannels[i], currentAngles[i]);
    }

    initialized = true;
    Serial.println("[ServoControl] 全部舵机初始化完成");
    return true;
}

bool ServoControl::setAngle(uint8_t index, float angleDeg) {
    if (!initialized || index >= SERVO_COUNT) {
        return false;
    }

    float clamped = constrain(angleDeg, 0.0f, 180.0f);
    ledcWrite(servoChannels[index], angleToDuty(clamped));
    currentAngles[index] = clamped;
    return true;
}

void ServoControl::setAllAngles(float angleDeg) {
    if (!initialized) {
        return;
    }

    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        setAngle(i, angleDeg);
    }
}

uint32_t ServoControl::angleToDuty(float angleDeg) const {
    float clamped = constrain(angleDeg, 0.0f, 180.0f);
    const uint32_t periodUs = 1000000UL / SERVO_PWM_FREQ_HZ;
    const uint32_t pulseUs = SERVO_MIN_PULSE_US +
                             (uint32_t)((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (clamped / 180.0f));
    const uint32_t maxDuty = (1UL << SERVO_PWM_RESOLUTION) - 1UL;

    return (pulseUs * maxDuty) / periodUs;
}
