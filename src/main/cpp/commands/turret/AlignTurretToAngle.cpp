// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/turret/AlignTurretToAngle.h"

AlignTurretToAngle::AlignTurretToAngle(std::function<units::degree_t()> angle, TurretSubsystem* turretSub)
    : angleToGoTo(angle), turretSubsystem(turretSub) {
    AddRequirements(turretSubsystem);
}

// Called when the command is initially scheduled.
void AlignTurretToAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AlignTurretToAngle::Execute() {
    turretSubsystem->SetTurretGoal(angleToGoTo());
}

// Called once the command ends or is interrupted.
void AlignTurretToAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignTurretToAngle::IsFinished() {
    return turretSubsystem->IsAtSetpoint();
}
