#pragma once

#include <frc/Servo.h>

class ContinuousServo : public frc::Servo {
public:
    explicit ContinuousServo(int channel);
    ContinuousServo(ContinuousServo&&) = default;
    ContinuousServo& operator=(ContinuousServo&&) = default;
};