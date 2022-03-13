// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "subsystems/DrivetrainSubsystem.h"
#include "commands/drive/FollowPath.h"

class MoveBackAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, MoveBackAuto> {
public:
    MoveBackAuto(DrivetrainSubsystem* drivetrainSub);

private:
    DrivetrainSubsystem* m_drivetrainSub;

    FollowPath moveBack{str::auto_consts::K_MAX_SPEED,
                        str::auto_consts::K_MAX_ACCEL,
                        frc::Pose2d{20_ft, 14.2_ft, frc::Rotation2d(0_deg)},
                        {frc::Translation2d(18_ft, 14.2_ft)},
                        frc::Pose2d(14_ft, 14.2_ft, frc::Rotation2d(0_deg)),
                        true,
                        m_drivetrainSub};
};
