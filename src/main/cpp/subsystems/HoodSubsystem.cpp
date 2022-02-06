// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/HoodSubsystem.h"
#include "str/Units.h"

HoodSubsystem::HoodSubsystem(str::ShooterLookupTable* shooterTable, VisionSubsystem* visionSub)
    : lookupTable(shooterTable), visionSubsystem(visionSub) {
    hoodEncoder.SetSamplesToAverage(50);
    hoodEncoder.SetMinRate(1.0);
    hoodEncoder.SetDistancePerPulse(1);
    hoodEncoder.SetReverseDirection(true);
    hoodController.SetSetpoint(0);
    hoodController.SetTolerance(str::shooter_pid::HOOD_TOLERANCE);
}

// This method will be called once per scheduler run
void HoodSubsystem::Periodic() {
    str::LookupValue goalTarget = lookupTable->Get(visionSubsystem->GetDistanceToTarget());
    hoodAngleToGoTo = goalTarget.angle;
    double hoodOutputVal = hoodController.Calculate(GetHoodAngle().to<double>());
    SetServoSpeed(hoodOutputVal);
}

int HoodSubsystem::ConvertHoodAngleToTicks(units::degree_t angle) {
    return str::Units::map(angle.to<double>(), 0, str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value(), 0,
                           str::shooter_pid::SHOOTER_HOOD_MAX_TICKS);
}

units::degree_t HoodSubsystem::ConvertHoodTicksToAngle(int ticks) {
    return units::degree_t(str::Units::map(ticks, 0, str::shooter_pid::SHOOTER_HOOD_MAX_TICKS, 0,
                                           str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value()));
}

units::degree_t HoodSubsystem::GetHoodAngle() {
    return ConvertHoodTicksToAngle(hoodEncoder.Get());
}

void HoodSubsystem::SetHoodToAngle(units::degree_t setpoint) {
    hoodController.SetSetpoint(setpoint.to<double>());
}

void HoodSubsystem::SetServoSpeed(double percent) {
    hoodServo.Set(std::clamp(percent, -1.0, 1.0));
}

units::degree_t HoodSubsystem::GetHoodAngleToGoTo() {
    return hoodAngleToGoTo;
}

bool HoodSubsystem::IsHoodAtSetpoint() {
    return hoodController.AtSetpoint();
}