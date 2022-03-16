// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/auto/ShootAndMoveBack.h"
#include "commands/auto/MoveBackAuto.h"
#include <frc2/command/ParallelCommandGroup.h>
#include "commands/shooter/SetShooterToGoal.h"
#include "commands/conveyor/FeedBallWait.h"
#include <frc2/command/ParallelRaceGroup.h>
#include "commands/drive/FollowPath.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ShootAndMoveBack::ShootAndMoveBack(DrivetrainSubsystem* drivetrainSub, ShooterSubsystem* shooterSub,
                                   TurretSubsystem* turretSub, VisionSubsystem* visionSub, HoodSubsystem* hoodSub,
                                   IntakeSubsystem* intakeSub, ConveyorSubsystem* conveyorSub)
    : drivetrainSubsystem(drivetrainSub),
      shooterSubsystem(shooterSub),
      turretSubsystem(turretSub),
      visionSubsystem(visionSub),
      hoodSubsystem(hoodSub),
      intakeSubsystem(intakeSub),
      conveyorSubsystem(conveyorSub) {
    AddCommands(SetShooterToGoal{shooterSub, visionSub, hoodSub, turretSub}.WithTimeout(5_s),
                FeedBallWait{[shooterSub] { return shooterSub->IsFlywheelUpToSpeed(); }, conveyorSub},
                std::move(moveBack));
    SetName("ShootAndMoveBack");
}
