// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetHoodToAngleAndWait.h"

SetHoodToAngleAndWait::SetHoodToAngleAndWait(std::function<units::degree_t()> angleFunc, HoodSubsystem* hoodSub)
    : angleFunction(angleFunc), hoodSubsystem(hoodSub) {
    AddRequirements(hoodSubsystem);
}

// Called when the command is initially scheduled.
void SetHoodToAngleAndWait::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SetHoodToAngleAndWait::Execute() {
    hoodSubsystem->SetHoodToAngle(angleFunction());
}

// Called once the command ends or is interrupted.
void SetHoodToAngleAndWait::End(bool interrupted) {}

// Returns true when the command should end.
bool SetHoodToAngleAndWait::IsFinished() {
    return hoodSubsystem->IsHoodAtSetpoint();
}
