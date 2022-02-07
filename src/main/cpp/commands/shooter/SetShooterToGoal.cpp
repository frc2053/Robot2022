// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetShooterToGoal.h"
#include "commands/shooter/SetSpeedAndWait.h"
#include "commands/shooter/SetHoodToAngleAndWait.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetShooterToGoal::SetShooterToGoal(ShooterSubsystem* shooterSub, VisionSubsystem* visionSub, HoodSubsystem* hoodSub)
    : shooterSubsystem(shooterSub), visionSubsystem(visionSub), hoodSubsystem(hoodSub) {
    // clang-format off
    AddCommands(
      SetHoodToAngleAndWait{[hoodSub, visionSub](){ return hoodSub->lookupTable->Get(visionSub->GetDistanceToTarget()).angle; }, hoodSub},
      SetSpeedAndWait{[hoodSub, visionSub](){ return hoodSub->lookupTable->Get(visionSub->GetDistanceToTarget()).rpm; }, shooterSubsystem}
    );
    // clang-format on
}
