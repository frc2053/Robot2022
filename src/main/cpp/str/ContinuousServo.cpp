#include "str/ContinuousServo.h"

ContinuousServo::ContinuousServo(int channel) : frc::Servo(channel) {
    SetBounds(1.800, 1.560, 1.545, 1.530, 1.300);
    SetPeriodMultiplier(frc::Servo::kPeriodMultiplier_1X);
    SetSpeed(0.0);
}