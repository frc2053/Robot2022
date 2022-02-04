#include "str/ContinuousServo.h"

ContinuousServo::ContinuousServo(int channel) : frc::Servo(channel) {
    SetBounds(1.9, 1.89, 1.550, 1.11, 1.1);
    SetPeriodMultiplier(frc::Servo::kPeriodMultiplier_4X);
    SetSpeed(0.0);
}