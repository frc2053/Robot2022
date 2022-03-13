// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <vector>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <subsystems/DrivetrainSubsystem.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include "commands/drive/BetterRamseteCommand.h"

class FollowPath : public frc2::CommandHelper<frc2::SequentialCommandGroup, FollowPath> {
public:
    FollowPath(units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAccel,
               const frc::Pose2d& startPt, const std::vector<frc::Translation2d>& middlePts, const frc::Pose2d& endPt,
               bool isReverse, DrivetrainSubsystem* swerve);
    units::meters_per_second_t m_maxSpeed;
    units::meters_per_second_squared_t m_maxAccel;
    const frc::Pose2d& m_startPt;
    const std::vector<frc::Translation2d>& m_middlePts;
    const frc::Pose2d& m_endPt;
    bool reverseDriving = false;

private:
    DrivetrainSubsystem* m_drivetrain;
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint{
        frc::SimpleMotorFeedforward<units::meters>(str::drive_pid::KS, str::drive_pid::KV, str::drive_pid::KA),
        str::drive_pid::DRIVE_KINEMATICS, 10_V};
};
