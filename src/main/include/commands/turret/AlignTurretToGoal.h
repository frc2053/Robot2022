// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandBase.h>
#include "subsystems/TurretSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"

class AlignTurretToGoal : public frc2::CommandHelper<frc2::CommandBase, AlignTurretToGoal> {
public:
    AlignTurretToGoal(TurretSubsystem* turretSub, VisionSubsystem* visionSub);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    TurretSubsystem* turretSubsystem;
    VisionSubsystem* visionSubsystem;
};
