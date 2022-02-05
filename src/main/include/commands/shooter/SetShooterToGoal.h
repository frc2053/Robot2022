// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/HoodSubsystem.h"

class SetShooterToGoal : public frc2::CommandHelper<frc2::ParallelCommandGroup, SetShooterToGoal> {
public:
    SetShooterToGoal(ShooterSubsystem* shooterSub, VisionSubsystem* visionSub, HoodSubsystem* hoodSub);

private:
    ShooterSubsystem* shooterSubsystem;
    VisionSubsystem* visionSubsystem;
    HoodSubsystem* hoodSubsystem;
};
