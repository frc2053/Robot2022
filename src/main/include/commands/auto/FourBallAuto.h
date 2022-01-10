// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/DrivetrainSubsystem.h>
#include <commands/drive/FollowPath.h>

class FourBallAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, FourBallAuto> {
   public:
    FourBallAuto(DrivetrainSubsystem* drivetrainSub);

   private:
    DrivetrainSubsystem* m_drivetrainSub;

    FollowPath turn1{str::auto_consts::K_MAX_SPEED,
                     str::auto_consts::K_MAX_ACCEL,
                     frc::Pose2d{25.82_ft, 6_ft, frc::Rotation2d(90_deg)},
                     {
                        frc::Translation2d(24.9_ft, 1_ft),
                        frc::Translation2d(16.9_ft, 6.2_ft)
                     },
                     frc::Pose2d(3.8_ft, 3.7_ft, frc::Rotation2d(45_deg)),
                     true,
                     m_drivetrainSub};
};
