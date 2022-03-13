// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ConveyorSubsystem.h"
#include "subsystems/DrivetrainSubsystem.h"
#include "commands/drive/FollowPath.h"

class ShootAndMoveBack : public frc2::CommandHelper<frc2::SequentialCommandGroup, ShootAndMoveBack> {
public:
    ShootAndMoveBack(DrivetrainSubsystem* drivetrainSub, ShooterSubsystem* shooterSub, TurretSubsystem* turretSub,
                     VisionSubsystem* visionSub, HoodSubsystem* hoodSub, IntakeSubsystem* intakeSub,
                     ConveyorSubsystem* conveyorSub);

private:
    DrivetrainSubsystem* drivetrainSubsystem;
    ShooterSubsystem* shooterSubsystem;
    TurretSubsystem* turretSubsystem;
    VisionSubsystem* visionSubsystem;
    HoodSubsystem* hoodSubsystem;
    IntakeSubsystem* intakeSubsystem;
    ConveyorSubsystem* conveyorSubsystem;

    FollowPath moveBack{str::auto_consts::K_MAX_SPEED,
                        str::auto_consts::K_MAX_ACCEL,
                        frc::Pose2d{20_ft, 14.2_ft, frc::Rotation2d(0_deg)},
                        {frc::Translation2d(18_ft, 14.2_ft)},
                        frc::Pose2d(14_ft, 14.2_ft, frc::Rotation2d(0_deg)),
                        true,
                        drivetrainSubsystem};
};
