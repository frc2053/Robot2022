// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive/FollowPath.h"
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
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
    // Create a voltage constraint to ensure we don't accelerate too fast
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            str::drive_pid::KS, str::drive_pid::KV, str::drive_pid::KA),
        str::drive_pid::DRIVE_KINEMATICS, 10_V);

    // Set up config for trajectory
    frc::TrajectoryConfig config(maxSpeed, maxAccel);
    //Change direction robot travels
    config.SetReversed(reverseDriving);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(str::drive_pid::DRIVE_KINEMATICS);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto trajectoryToFollow = frc::TrajectoryGenerator::GenerateTrajectory(
        startPt, middlePts, endPt, config);

    m_drivetrain->DrawTrajectory(trajectoryToFollow);

    frc2::RamseteCommand ramseteCommand(
        trajectoryToFollow, [this] { return m_drivetrain->GetPose(); },
        frc::RamseteController(str::auto_consts::K_RAMSETE_B,
                               str::auto_consts::K_RAMSETE_ZETA),
        frc::SimpleMotorFeedforward<units::meters>(
            str::drive_pid::KS, str::drive_pid::KV, str::drive_pid::KA),
        str::drive_pid::DRIVE_KINEMATICS,
        [this] { return m_drivetrain->GetWheelSpeeds(); },
        frc2::PIDController(str::drive_pid::KP_DRIVE_VEL, 0, 0),
        frc2::PIDController(str::drive_pid::KP_DRIVE_VEL, 0, 0),
        [this](auto left, auto right) {
            m_drivetrain->TankDriveVolts(left, right);
        },
        {m_drivetrain});

    AddCommands(frc2::InstantCommand([this, trajectoryToFollow, startPt]() {
                    m_drivetrain->ResetOdom(trajectoryToFollow.InitialPose());
                }),
                std::move(ramseteCommand),
                frc2::InstantCommand(
                    [this]() { m_drivetrain->TankDriveVolts(0_V, 0_V); },
                    {m_drivetrain}));
}
