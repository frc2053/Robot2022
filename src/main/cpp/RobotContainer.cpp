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
#include "commands/conveyor/FeedBalls.h"
#include "commands/shooter/SetShooterToGoalTele.h"
#include "wpi/PortForwarder.h"
#include "commands/turret/HomeTurret.h"
#include "commands/intake/IntakeBallTele.h"
#include "commands/intake/IntakeUpTele.h"
#include "commands/turret/AlignTurretToGoal.h"
#include "commands/conveyor/FeedBallWait.h"
#include "commands/conveyor/PukeBalls.h"
#include "commands/climber/WinchClimber.h"

RobotContainer::RobotContainer() {
    ConfigureButtonBindings();

    lookupTable.AddLookupValue(5.4_ft, str::LookupValue{4100_rpm, 0_deg});    // new
    lookupTable.AddLookupValue(6.8_ft, str::LookupValue{4350_rpm, 0_deg});
    lookupTable.AddLookupValue(7.0_ft, str::LookupValue{4000_rpm, 0_deg});    // new
    lookupTable.AddLookupValue(7.6_ft, str::LookupValue{4475_rpm, 0_deg});
    lookupTable.AddLookupValue(8.8_ft, str::LookupValue{4800_rpm, 0_deg});
    lookupTable.AddLookupValue(9.6_ft, str::LookupValue{4775_rpm, 0_deg});    // new
    lookupTable.AddLookupValue(10.5_ft, str::LookupValue{5100_rpm, 0_deg});
    lookupTable.AddLookupValue(11.7_ft, str::LookupValue{5100_rpm, 0_deg});
    lookupTable.AddLookupValue(12_ft, str::LookupValue{5200_rpm, 0_deg});
    lookupTable.AddLookupValue(12.7_ft, str::LookupValue{5400_rpm, 0_deg});
    lookupTable.AddLookupValue(13.9_ft, str::LookupValue{5900_rpm, 0_deg});
    lookupTable.AddLookupValue(15.0_ft, str::LookupValue{5200_rpm, 0_deg});    // new
    lookupTable.AddLookupValue(16_ft, str::LookupValue{6100_rpm, 0_deg});
    lookupTable.AddLookupValue(17.7_ft, str::LookupValue{6900_rpm, 0_deg});
    lookupTable.AddLookupValue(18.9_ft, str::LookupValue{6950_rpm, 0_deg});
    lookupTable.AddLookupValue(19.8_ft, str::LookupValue{6850_rpm, 0_deg});
    lookupTable.AddLookupValue(20.7_ft, str::LookupValue{6900_rpm, 0_deg});
    lookupTable.AddLookupValue(21.6_ft, str::LookupValue{6950_rpm, 0_deg});

    m_chooser.SetDefaultOption("Move Forward 5ft", &moveForwardAuto);
    m_chooser.AddOption("Move Back Auto", &moveBackAuto);
    m_chooser.AddOption("Shoot And Move Back", &shootAndMoveBack);
    m_chooser.AddOption("Two Ball Auto", &twoBallAuto);
    m_chooser.AddOption("Three Ball Auto", &threeBallAuto);

    frc::SmartDashboard::PutData(&m_chooser);

    TeleopDrive driveCmd = TeleopDrive([this]() { return speedLimiter.Calculate(-m_driverController.GetLeftY()); },
                                       [this]() { return m_driverController.GetRightX(); },
                                       [this]() { return m_driverController.GetRightBumper(); }, &drivetrainSubsystem);

    drivetrainSubsystem.SetDefaultCommand(driveCmd);

    WinchClimber winchCmd =
        WinchClimber([this]() { return m_operatorController.GetRightY(); },
                     [this]() { return m_operatorController.GetRightBumperPressed(); }, &climberSubsystem);

    climberSubsystem.SetDefaultCommand(winchCmd);

    frc::SmartDashboard::PutData("Zero Yaw", new frc2::InstantCommand([this] { drivetrainSubsystem.ResetGyro(); }));

    frc::SmartDashboard::PutData("Reset Odom",
                                 new frc2::InstantCommand([this] { drivetrainSubsystem.ResetOdom(frc::Pose2d()); }));

    frc::SmartDashboard::PutData("Home Turret", new HomeTurret(&turretSubsystem));

    wpi::PortForwarder::GetInstance().Add(5800, "10.20.53.105", 5800);
    wpi::PortForwarder::GetInstance().Add(1181, "10.20.53.105", 1181);
    wpi::PortForwarder::GetInstance().Add(1182, "10.20.53.105", 1182);
}

void RobotContainer::ConfigureButtonBindings() {
    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kRightBumper)
        .WhenPressed(frc2::InstantCommand(
            [this] {
                shooterSubsystem.SetShooterSpeed(
                    units::revolutions_per_minute_t(frc::SmartDashboard::GetNumber("Shooter Speed To Go To (RPM)", 0)));
                hoodSubsystem.SetHoodToAngle(
                    units::degree_t(frc::SmartDashboard::GetNumber("Shooter Hood Angle To Go To (Degrees)", 0)));
            },
            {&shooterSubsystem, &hoodSubsystem}));

    // frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kRightBumper)
    //     .WhenReleased(frc2::InstantCommand(
    //         [this] {
    //             shooterSubsystem.SetShooterSpeed(0_rpm);
    //             hoodSubsystem.SetHoodToAngle(0_deg);
    //         },
    //         {&shooterSubsystem, &hoodSubsystem}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX)
        .WhenHeld(IntakeBallTele(&intakeSubsystem, &conveyorSubsystem, &visionSubsystem));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kX)
        .WhenReleased(IntakeUpTele(&intakeSubsystem, &conveyorSubsystem));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kA)
        .WhenHeld(FeedBalls(&conveyorSubsystem));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kY)
        .WhileHeld(PukeBalls(&conveyorSubsystem, &intakeSubsystem));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kStart)
        .WhenPressed(frc2::InstantCommand([this] { climberSubsystem.ExtendClimber(); }, {&climberSubsystem}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kBack)
        .WhenPressed(frc2::InstantCommand(
            [this] {
                climberSubsystem.RetractClimber();
                turretSubsystem.LockTurret();
            },
            {&climberSubsystem}));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kLeftStick)
        .WhenPressed(frc2::InstantCommand([this] { climberSubsystem.LockClimber(); }));    // unlock

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kRightStick)
        .WhenPressed(frc2::InstantCommand([this] { climberSubsystem.UnlockClimber(); }));    // lock

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kLeftBumper)
        .WhenHeld(SetShooterToGoalTele(&shooterSubsystem, &visionSubsystem, &hoodSubsystem, &turretSubsystem));

    frc2::JoystickButton(&m_operatorController, frc::XboxController::Button::kLeftBumper)
        .WhenReleased(SetShooterSpeed([] { return 0_rpm; }, &shooterSubsystem));

    auto dumpBallButton = frc2::Trigger([this]() { return m_operatorController.GetLeftTriggerAxis() > .2; });

    dumpBallButton.WhileActiveOnce(frc2::InstantCommand(
        [this] {
            shooterSubsystem.SetShooterSpeed(1200_rpm);
            hoodSubsystem.SetHoodToAngle(45_deg);
        },
        {&shooterSubsystem, &hoodSubsystem}));

    dumpBallButton.WhenInactive(frc2::InstantCommand(
        [this] {
            shooterSubsystem.SetShooterSpeed(0_rpm);
            hoodSubsystem.SetHoodToAngle(0_deg);
        },
        {&shooterSubsystem, &hoodSubsystem}));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_chooser.GetSelected();
}

DrivetrainSubsystem& RobotContainer::GetRobotDriveSubsystem() {
    return drivetrainSubsystem;
}

IntakeSubsystem& RobotContainer::GetIntakeSubsystem() {
    return intakeSubsystem;
}

ConveyorSubsystem& RobotContainer::GetConveyorSubsystem() {
    return conveyorSubsystem;
}

TurretSubsystem& RobotContainer::GetTurretSubsystem() {
    return turretSubsystem;
}

VisionSubsystem& RobotContainer::GetVisionSubsystem() {
    return visionSubsystem;
}

ShooterSubsystem& RobotContainer::GetShooterSubsystem() {
    return shooterSubsystem;
}

HoodSubsystem& RobotContainer::GetHoodSubsystem() {
    return hoodSubsystem;
}

ClimberSubsystem& RobotContainer::GetClimberSubsystem() {
    return climberSubsystem;
}