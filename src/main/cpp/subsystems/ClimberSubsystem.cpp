// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() {}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

void ClimberSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig;
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled;
    climberMotor.ConfigAllSettings(baseConfig);

    climberMotor.ConfigReverseSoftLimitEnable(true);
    climberMotor.ConfigReverseSoftLimitThreshold(-175119);
}

void ClimberSubsystem::ExtendClimber() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void ClimberSubsystem::RetractClimber() {
    climberSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void ClimberSubsystem::LockClimber() {
    climberLockSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void ClimberSubsystem::UnlockClimber() {
    climberLockSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void ClimberSubsystem::SetClimberSpeed(double outputSpeed) {
    climberMotor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, outputSpeed);
}