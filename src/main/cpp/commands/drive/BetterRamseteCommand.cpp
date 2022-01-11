// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive/BetterRamseteCommand.h"

BetterRamseteCommand::BetterRamseteCommand(frc::Trajectory traj,
                                           std::function<frc::Pose2d()> pose,
                                           DrivetrainSubsystem* driveSub)
    : trajToFollow(traj), m_pose(pose), drivetrainSubsystem(driveSub) {
    AddRequirements(drivetrainSubsystem);
    SetName("BetterRamseteCommand");
}

// Called when the command is initially scheduled.
void BetterRamseteCommand::Initialize() {
    m_prevTime = -1_s;
    auto initialState = trajToFollow.Sample(0_s);
    m_prevSpeeds = str::drive_pid::DRIVE_KINEMATICS.ToWheelSpeeds(
        frc::ChassisSpeeds{initialState.velocity, 0_mps,
                           initialState.velocity * initialState.curvature});
    m_timer.Reset();
    m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void BetterRamseteCommand::Execute() {
    units::second_t timeElapsed = m_timer.Get();
    units::second_t dt = timeElapsed - m_prevTime;

    if (m_prevTime < 0_s) {
        drivetrainSubsystem->TankDriveVolts(0_V, 0_V);
        m_prevTime = timeElapsed;
        return;
    }

    auto desiredPose = trajToFollow.Sample(timeElapsed);

    auto refChassisSpeeds = m_ramseteController.Calculate(m_pose(), desiredPose);

    frc::DifferentialDriveWheelSpeeds wheelSpeeds =
        str::drive_pid::DRIVE_KINEMATICS.ToWheelSpeeds(refChassisSpeeds);

    units::volt_t feedForwardLeft = feedForward.Calculate(wheelSpeeds.left, (wheelSpeeds.left - m_prevSpeeds.left) / dt);
    units::volt_t feedForwardRight = feedForward.Calculate(wheelSpeeds.right, (wheelSpeeds.right - m_prevSpeeds.right) / dt);

    drivetrainSubsystem->TankDriveVelocity(wheelSpeeds.left, wheelSpeeds.right, feedForwardLeft, feedForwardRight);

    m_prevSpeeds = wheelSpeeds;
    m_prevTime = timeElapsed;
}

// Called once the command ends or is interrupted.
void BetterRamseteCommand::End(bool interrupted) {
    m_timer.Stop();
    drivetrainSubsystem->TankDriveVolts(0_V, 0_V);
}

// Returns true when the command should end.
bool BetterRamseteCommand::IsFinished() {
    return m_timer.HasElapsed(trajToFollow.TotalTime());
}
