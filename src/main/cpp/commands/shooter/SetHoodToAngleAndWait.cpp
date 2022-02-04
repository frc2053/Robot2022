// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetHoodToAngleAndWait.h"

SetHoodToAngleAndWait::SetHoodToAngleAndWait(std::function<units::degree_t()> angleFunc, ShooterSubsystem* shooterSub)
    : angleFunction(angleFunc), shooterSubsystem(shooterSub) {
    AddRequirements(shooterSubsystem);
}

// Called when the command is initially scheduled.
void SetHoodToAngleAndWait::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetHoodToAngleAndWait::Execute() {
    shooterSubsystem->SetHoodToAngle(angleFunction());
}

// Called once the command ends or is interrupted.
void SetHoodToAngleAndWait::End(bool interrupted) {}

// Returns true when the command should end.
bool SetHoodToAngleAndWait::IsFinished() {
    return shooterSubsystem->IsHoodAtSetpoint();
}
