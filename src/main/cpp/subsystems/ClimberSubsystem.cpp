// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() {}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {}

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