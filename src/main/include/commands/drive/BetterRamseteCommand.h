// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/controller/RamseteController.h>
#include "subsystems/DrivetrainSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BetterRamseteCommand
    : public frc2::CommandHelper<frc2::CommandBase, BetterRamseteCommand> {
public:
    BetterRamseteCommand(frc::Trajectory traj, std::function<frc::Pose2d()> pose, DrivetrainSubsystem* driveSub);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
private:
    frc::Timer m_timer;
    frc::Trajectory trajToFollow;
    frc::RamseteController m_ramseteController;
    std::function<frc::Pose2d()> m_pose;
    DrivetrainSubsystem* drivetrainSubsystem;
    frc::SimpleMotorFeedforward<units::meters> feedForward{
        str::drive_pid::KS, 
        str::drive_pid::KV, 
        str::drive_pid::KA
    };
    units::second_t m_prevTime;
    frc::DifferentialDriveWheelSpeeds m_prevSpeeds;
};
