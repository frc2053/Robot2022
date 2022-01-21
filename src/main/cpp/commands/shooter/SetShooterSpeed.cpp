// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetShooterSpeed.h"

SetShooterSpeed::SetShooterSpeed(std::function<double()> speed, ShooterSubsystem* shooterSub) : speedFunc(speed), shooterSubsystem(shooterSub) {
  AddRequirements(shooterSubsystem);
}

// Called when the command is initially scheduled.
void SetShooterSpeed::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetShooterSpeed::Execute() {
  shooterSubsystem->SetShooterSpeedPercent(speedFunc());
}

// Called once the command ends or is interrupted.
void SetShooterSpeed::End(bool interrupted) {}

// Returns true when the command should end.
bool SetShooterSpeed::IsFinished() {
  return false;
}
