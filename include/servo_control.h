/**
 * @file servo_control.h
 * @brief 180°舵机PWM控制模块
 */

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>
#include "config.h"

class ServoControl {
public:
    ServoControl();

    bool begin();
    bool setAngle(uint8_t index, float angleDeg);
    void setAllAngles(float angleDeg);
    uint8_t getServoCount() const { return SERVO_COUNT; }

private:
    bool initialized;
    float currentAngles[SERVO_COUNT];

    static uint8_t servoPins[SERVO_COUNT];
    static uint8_t servoChannels[SERVO_COUNT];

    uint32_t angleToDuty(float angleDeg) const;
};

extern ServoControl servoControl;

#endif // SERVO_CONTROL_H
