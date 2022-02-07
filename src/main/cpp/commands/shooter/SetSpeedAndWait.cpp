// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetSpeedAndWait.h"
#include <iostream>

SetSpeedAndWait::SetSpeedAndWait(std::function<units::revolutions_per_minute_t()> speed, ShooterSubsystem* shooterSub)
    : speedFunc(speed), shooterSubsystem(shooterSub) {
    AddRequirements(shooterSubsystem);
}

// Called when the command is initially scheduled.
void SetSpeedAndWait::Initialize() {
    std::cout << "Inited set speed and wait\n";
}

// Called repeatedly when this Command is scheduled to run
void SetSpeedAndWait::Execute() {
    shooterSubsystem->SetShooterSpeed(speedFunc());
}

// Called once the command ends or is interrupted.
void SetSpeedAndWait::End(bool interrupted) {
    std::cout << "Ended set speed and wait\n";
}

// Returns true when the command should end.
bool SetSpeedAndWait::IsFinished() {
    return shooterSubsystem->IsFlywheelUpToSpeed();
}
