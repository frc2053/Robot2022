// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() = default;

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void IntakeSubsystem::PutIntakeDown() {
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1);
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void IntakeSubsystem::PutIntakeUp() {
    intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void IntakeSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonSRXConfiguration baseConfig;
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    intakeMotor.ConfigAllSettings(baseConfig);
}
