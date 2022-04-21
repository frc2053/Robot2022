// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/auto/ThreeBallAuto.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "commands/shooter/SetShooterSpeed.h"
#include "commands/turret/HomeTurret.h"
#include "commands/intake/IntakeABall.h"
#include "commands/shooter/SetShooterToGoal.h"
#include "commands/conveyor/FeedBallWait.h"
#include "commands/turret/AlignTurretToAngle.h"
#include <frc2/command/ParallelRaceGroup.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ThreeBallAuto::ThreeBallAuto(DrivetrainSubsystem* drivetrainSub, ShooterSubsystem* shooterSub,
                             TurretSubsystem* turretSub, VisionSubsystem* visionSub, HoodSubsystem* hoodSub,
                             IntakeSubsystem* intakeSub, ConveyorSubsystem* conveyorSub)
    : m_drivetrainSub(drivetrainSub),
      m_shooterSub(shooterSub),
      m_turretSub(turretSub),
      m_visionSub(visionSub),
      m_hoodSub(hoodSub),
      m_intakeSub(intakeSub),
      m_conveyorSub(conveyorSub) {
    // clang-format off
    AddCommands(
        frc2::InstantCommand{[this]() { m_drivetrainSub->SetGyroOffset(-90_deg); }},
        frc2::ParallelCommandGroup{std::move(toSecondBallPath), IntakeABall{m_intakeSub, m_conveyorSub}.WithTimeout(6_s), AlignTurretToAngle{[]{ return -75_deg; }, m_turretSub}},
        SetShooterToGoal{m_shooterSub, m_visionSub, m_hoodSub, m_turretSub},
        FeedBallWait{[this]{ return m_shooterSub->IsFlywheelUpToSpeed(); }, m_conveyorSub},
        frc2::ParallelCommandGroup{std::move(toThirdBallPath), IntakeABall{m_intakeSub, m_conveyorSub}.WithTimeout(6_s), AlignTurretToAngle{[]{ return -75_deg; }, m_turretSub}},
        SetShooterToGoal{m_shooterSub, m_visionSub, m_hoodSub, m_turretSub},
        FeedBallWait{[this]{ return m_shooterSub->IsFlywheelUpToSpeed(); }, m_conveyorSub}
    );
    // clang-format on
    SetName("ThreeBallAuto");
}