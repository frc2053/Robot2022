// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

TurretSubsystem::TurretSubsystem() {
    ConfigureMotors();
    frc::SmartDashboard::PutData("Turret Sim", &turretViz);
}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {}

void TurretSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonSRXConfiguration baseConfig;
    baseConfig.primaryPID.selectedFeedbackSensor =
        ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    turretMotor.ConfigAllSettings(baseConfig);
}