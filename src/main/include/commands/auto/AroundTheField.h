// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/DrivetrainSubsystem.h>
#include <commands/drive/FollowPath.h>

class AroundTheField
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, AroundTheField> {
   public:
    AroundTheField(DrivetrainSubsystem* drivetrainSub);

   private:
    DrivetrainSubsystem* m_drivetrainSub;

    FollowPath turn1{str::auto_consts::K_MAX_SPEED,
                     str::auto_consts::K_MAX_ACCEL,
                     frc::Pose2d{0_ft, 13_ft, frc::Rotation2d(90_deg)},
                     {frc::Translation2d(3_ft, 22_ft)},
                     frc::Pose2d(27_ft, 27_ft, frc::Rotation2d(0_deg)),
                     m_drivetrainSub};
    FollowPath turn2{str::auto_consts::K_MAX_SPEED,
                     str::auto_consts::K_MAX_ACCEL,
                     frc::Pose2d(27_ft, 27_ft, frc::Rotation2d(0_deg)),
                     {frc::Translation2d(52_ft, 22_ft)},
                     frc::Pose2d(54_ft, 13_ft, frc::Rotation2d(-90_deg)),
                     m_drivetrainSub};
};
