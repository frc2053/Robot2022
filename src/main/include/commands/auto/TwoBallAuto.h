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

class TwoBallAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, TwoBallAuto> {
public:
    TwoBallAuto(DrivetrainSubsystem* drivetrainSub, ShooterSubsystem* shooterSub, TurretSubsystem* turretSub,
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

    FollowPath toSecondBallPath{6_fps,
                                str::auto_consts::K_MAX_ACCEL,
                                frc::Pose2d(20.553_ft, 17.448_ft, frc::Rotation2d(135_deg)),
                                {},
                                frc::Pose2d(14.944_ft, 17.292_ft, frc::Rotation2d(-90_deg)),
                                false,
                                m_drivetrainSub};
};
