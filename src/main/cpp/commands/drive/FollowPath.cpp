// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive/FollowPath.h"
#include <frc/trajectory/TrajectoryGenerator.h>
#include "commands/drive/BetterRamseteCommand.h"
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/Field2d.h>

FollowPath::FollowPath(units::meters_per_second_t maxSpeed,
                       units::meters_per_second_squared_t maxAccel,
                       const frc::Pose2d& startPt,
                       const std::vector<frc::Translation2d>& middlePts,
                       const frc::Pose2d& endPt, bool isReverse,
                       DrivetrainSubsystem* drivetrain)
    : m_maxSpeed(maxSpeed),
      m_maxAccel(maxAccel),
      m_startPt(startPt),
      m_middlePts(middlePts),
      m_endPt(endPt),
      reverseDriving(isReverse),
      m_drivetrain(drivetrain) {

    // Set up config for trajectory
    frc::TrajectoryConfig config(maxSpeed, maxAccel);
    //Change direction robot travels
    config.SetReversed(reverseDriving);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(str::drive_pid::DRIVE_KINEMATICS);
    // Apply the voltage constraint
    //config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto trajectoryToFollow = frc::TrajectoryGenerator::GenerateTrajectory(
        startPt, middlePts, endPt, config);

    m_drivetrain->DrawTrajectory(trajectoryToFollow);

    BetterRamseteCommand ramseteCommand(
        trajectoryToFollow, 
        [this] { return m_drivetrain->GetPose(); },
        m_drivetrain
    );

    AddCommands(frc2::InstantCommand([this, trajectoryToFollow, startPt]() {
                    m_drivetrain->ResetOdom(trajectoryToFollow.InitialPose());
                }),
                std::move(ramseteCommand),
                frc2::InstantCommand(
                    [this]() { m_drivetrain->TankDriveVolts(0_V, 0_V); },
                    {m_drivetrain}));
}
