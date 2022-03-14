// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/turret/AlignTurretToGoal.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AlignTurretToGoal::AlignTurretToGoal(TurretSubsystem* turretSub, VisionSubsystem* visionSub)
    : turretSubsystem(turretSub), visionSubsystem(visionSub) {
    AddRequirements({turretSubsystem, visionSubsystem});
    SetName("AlignTurretToGoal");
}

// Called when the command is initially scheduled.
void AlignTurretToGoal::Initialize() {}

void AlignTurretToGoal::Execute() {
    turretSubsystem->SetTurretGoal(-visionSubsystem->GetYawToTarget() + turretSubsystem->GetCurrentTurretAngle());
}

void AlignTurretToGoal::End(bool interrupted) {}

bool AlignTurretToGoal::IsFinished() {
    return turretSubsystem->IsAtSetpoint();
}
