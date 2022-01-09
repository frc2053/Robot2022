// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "commands/drive/TurnToAngle.h"
#include <frc2/command/ParallelRaceGroup.h>

RobotContainer::RobotContainer() {
    ConfigureButtonBindings();

    m_chooser.SetDefaultOption("Around the Field", &aroundTheFieldAuto);

    frc::SmartDashboard::PutData(&m_chooser);

    drivetrainSubsystem.SetDefaultCommand(frc2::RunCommand(
        [this] {
            drivetrainSubsystem.CurvatureDrive(
                -m_driverController.GetLeftY(), m_driverController.GetRightX(),
                m_driverController.GetRightBumper());
        },
        {&drivetrainSubsystem}));
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
        .WhenPressed(
            TurnToAngle{90_deg, &drivetrainSubsystem}.WithInterrupt([this]() {
                return std::abs(m_driverController.GetLeftY()) > .2 ||
                       std::abs(m_driverController.GetRightX()) > .2;
            }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}

const DrivetrainSubsystem& RobotContainer::GetRobotDriveSubsystem() const {
    return drivetrainSubsystem;
}
