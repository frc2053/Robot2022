// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <subsystems/DrivetrainSubsystem.h>
#include <commands/drive/FollowPath.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ConveyorSubsystem.h"

class FourBallAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, FourBallAuto> {
public:
    FourBallAuto(DrivetrainSubsystem* drivetrainSub, ShooterSubsystem* shooterSub, TurretSubsystem* turretSub,
                 VisionSubsystem* visionSub, HoodSubsystem* hoodSub, IntakeSubsystem* intakeSub,
                 ConveyorSubsystem* conveyorSub);

private:
    DrivetrainSubsystem* m_drivetrainSub;
    ShooterSubsystem* m_shooterSub;
    TurretSubsystem* m_turretSub;
    VisionSubsystem* m_visionSub;
    HoodSubsystem* m_hoodSub;
    IntakeSubsystem* m_intakeSub;
    ConveyorSubsystem* m_conveyorSub;

    FollowPath toSecondBallPath{str::auto_consts::K_MAX_SPEED,
                                str::auto_consts::K_MAX_ACCEL,
                                frc::Pose2d(24.346_ft, 5.322_ft, frc::Rotation2d(90_deg)),
                                {},
                                frc::Pose2d(25.018_ft, 0.796_ft, frc::Rotation2d(90_deg)),
                                true,
                                m_drivetrainSub};
    FollowPath toThirdAndFourthBallPath{str::auto_consts::K_MAX_SPEED,
                                        str::auto_consts::K_MAX_ACCEL,
                                        frc::Pose2d(25.018_ft, 0.796_ft, frc::Rotation2d(90_deg)),
                                        {frc::Translation2d(16.694_ft, 6.221_ft)},
                                        frc::Pose2d(3.554_ft, 3.792_ft, frc::Rotation2d(45_deg)),
                                        true,
                                        m_drivetrainSub};
};
