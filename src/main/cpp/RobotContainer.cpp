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
#include "commands/drive/TeleopDrive.h"
#include <frc2/command/RunCommand.h>
#include "str/Units.h"
#include "commands/intake/IntakeABall.h"
#include "commands/shooter/SetShooterSpeed.h"
#include "commands/shooter/SetSpeedAndWait.h"

RobotContainer::RobotContainer() {
    ConfigureButtonBindings();

    m_chooser.SetDefaultOption("Move Forward 5ft", &moveForwardAuto);
    //m_chooser.AddOption("Four Ball Auto", &fourBallAuto);

    frc::SmartDashboard::PutData(&m_chooser);

    TeleopDrive driveCmd = TeleopDrive([this]() { return -m_driverController.GetLeftY(); },
                                       [this]() { return m_driverController.GetRightX(); },
                                       [this]() { return m_driverController.GetRightBumper(); }, &drivetrainSubsystem);

    drivetrainSubsystem.SetDefaultCommand(driveCmd);

    frc::SmartDashboard::PutData("Zero Yaw", new frc2::InstantCommand([this] { drivetrainSubsystem.ResetGyro(); }));

    frc::SmartDashboard::PutData("Reset Odom",
                                 new frc2::InstantCommand([this] { drivetrainSubsystem.ResetOdom(frc::Pose2d()); }));
}

void RobotContainer::ConfigureButtonBindings() {
    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    //     .WhenPressed(TurnToAngle([]() { return units::degree_t(90); }, &drivetrainSubsystem).WithInterrupt([this]() {
    //         return std::abs(m_driverController.GetLeftY()) > .2 || std::abs(m_driverController.GetRightX()) > .2;
    //     }));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
        .WhenPressed(frc2::InstantCommand(
            [this] { shooterSubsystem.SetShooterSpeed(shooterSubsystem.GetShooterSetpoint() - 100_rpm); },
            {&shooterSubsystem}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
        .WhenPressed(frc2::InstantCommand(
            [this] { shooterSubsystem.SetShooterSpeed(shooterSubsystem.GetShooterSetpoint() + 100_rpm); },
            {&shooterSubsystem}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
        .WhenPressed(frc2::InstantCommand(
            [this] {
                intakeSubsystem.PutIntakeDown();
                conveyorSubsystem.SetFunnelSpeed(1);
                conveyorSubsystem.SetConveyorSpeed(-1);
                //shooterSubsystem.SetShooterSpeed(6000_rpm);
            },
            {&intakeSubsystem, &conveyorSubsystem, &shooterSubsystem}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
        .WhenReleased(frc2::InstantCommand(
            [this] {
                intakeSubsystem.PutIntakeUp();
                conveyorSubsystem.SetFunnelSpeed(0);
                conveyorSubsystem.SetConveyorSpeed(0);
                //shooterSubsystem.SetShooterSpeed(0_rpm);
            },
            {&intakeSubsystem, &conveyorSubsystem, &shooterSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
    //     .WhenPressed(frc2::InstantCommand([this] { shooterSubsystem.SetShooterSpeed(0_rpm); }, {&shooterSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
    //     .WhenPressed(frc2::InstantCommand(
    //         [this] {
    //             shooterSubsystem.SetShooterSurfaceSpeed(
    //                 str::Units::ConvertAngularVelocityToLinearVelocity(shooterSubsystem.GetShooterSetpoint(),
    //                                                                    str::physical_dims::SHOOTER_WHEEL_DIAMETER /
    //                                                                    2) -
    //                 2_fps);
    //         },
    //         {&shooterSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
    //     .WhenPressed(frc2::InstantCommand(
    //         [this] {
    //             shooterSubsystem.SetShooterSurfaceSpeed(
    //                 str::Units::ConvertAngularVelocityToLinearVelocity(shooterSubsystem.GetShooterSetpoint(),
    //                                                                    str::physical_dims::SHOOTER_WHEEL_DIAMETER /
    //                                                                    2) +
    //                 2_fps);
    //         },
    //         {&shooterSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
    //     .WhenPressed(SetSpeedAndWait([]() { return 3000_rpm; }, &shooterSubsystem));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    //     .WhenPressed(frc2::InstantCommand(
    //         [this] { turretSubsystem.SetTurretGoal(turretSubsystem.GetTurretSetpoint() - 10_deg); },
    //         {&turretSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
    //     .WhenPressed(frc2::InstantCommand(
    //         [this] { turretSubsystem.SetTurretGoal(turretSubsystem.GetTurretSetpoint() + 10_deg); },
    //         {&turretSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    //     .WhenHeld(frc2::RunCommand([this] { turretSubsystem.SetTurretGoal(visionSubsystem.GetYawToTarget()); },
    //                                {&turretSubsystem}));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper)
    //     .WhenReleased(frc2::RunCommand([this] { turretSubsystem.SetTurretGoal(0_deg); }, {&turretSubsystem}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}

const DrivetrainSubsystem& RobotContainer::GetRobotDriveSubsystem() const {
    return drivetrainSubsystem;
}

const ShooterSubsystem& RobotContainer::GetShooterSubsystem() const {
    return shooterSubsystem;
}

// const TurretSubsystem& RobotContainer::GetTurretSubsystem() const {
//     return turretSubsystem;
// }