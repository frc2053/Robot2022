// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive/FollowPath.h"
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/Field2d.h>
#include <fstream>
#include <frc/DriverStation.h>

FollowPath::FollowPath(units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAccel,
                       const frc::Pose2d& startPt, const std::vector<frc::Translation2d>& middlePts,
                       const frc::Pose2d& endPt, bool isReverse, DrivetrainSubsystem* drivetrain)
    : m_maxSpeed(maxSpeed),
      m_maxAccel(maxAccel),
      m_startPt(startPt),
      m_middlePts(middlePts),
      m_endPt(endPt),
      reverseDriving(isReverse),
      m_drivetrain(drivetrain) {
    SetName("FollowPath");

    // Set up config for trajectory
    frc::TrajectoryConfig config(maxSpeed, maxAccel);
    // Change direction robot travels
    config.SetReversed(reverseDriving);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(str::drive_pid::DRIVE_KINEMATICS);
    // Apply the voltage constraint
    config.AddConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    auto trajectoryToFollow = frc::TrajectoryGenerator::GenerateTrajectory(startPt, middlePts, endPt, config);

    std::ofstream trajFile;
    trajFile.open("/home/lvuser/trajFileOut.csv");
    for (const auto& state : trajectoryToFollow.States()) {
        trajFile << state.t.value() << "," << state.pose.X().value() << "," << state.pose.Y().value() << ","
                 << state.pose.Rotation().Degrees().value() << "\n";
    }
    trajFile.close();

    BetterRamseteCommand cmd = BetterRamseteCommand(
        trajectoryToFollow, [this] { return m_drivetrain->GetPose(); }, m_drivetrain);

    AddCommands(frc2::InstantCommand([this]() {
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
                        m_drivetrain->SetGyroOffset(180_deg);
                    } else {
                        m_drivetrain->SetGyroOffset(0_deg);
                    }
                }),
                cmd, frc2::InstantCommand([this]() { m_drivetrain->TankDriveVolts(0_V, 0_V); }, {m_drivetrain}));
}
