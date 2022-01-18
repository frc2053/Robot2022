// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ConveyorSubsystem.h"

ConveyorSubsystem::ConveyorSubsystem() {
    SetName("ConveyorSubsystem");
    ConfigureMotors();
}

// This method will be called once per scheduler run
void ConveyorSubsystem::Periodic() {
    double rawBottomDist = bottomConveyorSensor.GetRange();
    double rawTopDist = topConveyorSensor.GetRange();
    bottomDistFiltered = units::millimeter_t(bottomFilter.Calculate(rawBottomDist));
    topDistFiltered = units::millimeter_t(topFilter.Calculate(rawTopDist));
}

void ConveyorSubsystem::SetBottomConveyorSpeed(double speed) {
    bottomConveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void ConveyorSubsystem::SetTopConveyorSpeed(double speed) {
    topConveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

bool ConveyorSubsystem::DoesTopSensorSeeBall() {
    return topDistFiltered < str::intake_vars::DIST_THRESHOLD;
}

bool ConveyorSubsystem::DoesBottomSensorSeeBall() {
    return bottomDistFiltered < str::intake_vars::DIST_THRESHOLD;
}

void ConveyorSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonSRXConfiguration baseConfig;
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    bottomConveyorMotor.ConfigAllSettings(baseConfig);
    topConveyorMotor.ConfigAllSettings(baseConfig);
}