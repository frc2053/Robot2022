// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetShooterSpeed.h"

SetShooterSpeed::SetShooterSpeed(std::function<units::revolutions_per_minute_t()> speed, ShooterSubsystem* shooterSub) : speedFunc(speed), shooterSubsystem(shooterSub) {
  AddRequirements(shooterSubsystem);
}

// Called when the command is initially scheduled.
void SetShooterSpeed::Initialize() {
    shooterSubsystem->SetShooterSpeed(speedFunc());
}