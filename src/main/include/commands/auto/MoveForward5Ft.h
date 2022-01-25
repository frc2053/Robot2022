// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/DrivetrainSubsystem.h"
#include "commands/drive/FollowPath.h"

class MoveForward5Ft : public frc2::CommandHelper<frc2::SequentialCommandGroup, MoveForward5Ft> {
public:
    MoveForward5Ft(DrivetrainSubsystem* drivetrainSub);

private:
    DrivetrainSubsystem* m_drivetrainSub;

    FollowPath moveforward{str::auto_consts::K_MAX_SPEED,
                           str::auto_consts::K_MAX_ACCEL,
                           frc::Pose2d{0_ft, 0_ft, frc::Rotation2d(0_deg)},
                           {frc::Translation2d(0_ft, 2.5_ft)},
                           frc::Pose2d(0_ft, 5_ft, frc::Rotation2d(0_deg)),
                           false,
                           m_drivetrainSub};
};
