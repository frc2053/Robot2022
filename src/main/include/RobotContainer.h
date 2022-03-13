// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>

#include "commands/auto/FourBallAuto.h"
#include "commands/auto/MoveForward5Ft.h"
#include "Constants.h"
#include <frc/filter/SlewRateLimiter.h>
#include "subsystems/DrivetrainSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ConveyorSubsystem.h"
#include "subsystems/TurretSubsystem.h"
#include "subsystems/HoodSubsystem.h"
#include "str/ShooterLookupTable.h"
#include "subsystems/ClimberSubsystem.h"
#include "commands/auto/MoveBackAuto.h"

class RobotContainer {
public:
    RobotContainer();
    frc2::Command* GetAutonomousCommand();
    DrivetrainSubsystem& GetRobotDriveSubsystem();
    IntakeSubsystem& GetIntakeSubsystem();
    ConveyorSubsystem& GetConveyorSubsystem();
    TurretSubsystem& GetTurretSubsystem();
    VisionSubsystem& GetVisionSubsystem();
    ShooterSubsystem& GetShooterSubsystem();
    HoodSubsystem& GetHoodSubsystem();
    ClimberSubsystem& GetClimberSubsystem();

private:
    void ConfigureButtonBindings();

    str::ShooterLookupTable lookupTable;

    DrivetrainSubsystem drivetrainSubsystem;
    IntakeSubsystem intakeSubsystem{};
    ConveyorSubsystem conveyorSubsystem{};
    TurretSubsystem turretSubsystem{};
    VisionSubsystem visionSubsystem{&drivetrainSubsystem, &turretSubsystem};
    ShooterSubsystem shooterSubsystem{&visionSubsystem, &lookupTable};
    HoodSubsystem hoodSubsystem{&lookupTable, &visionSubsystem};
    ClimberSubsystem climberSubsystem{};

    frc::SendableChooser<frc2::Command*> m_chooser;
    FourBallAuto fourBallAuto{&drivetrainSubsystem, &shooterSubsystem, &turretSubsystem,  &visionSubsystem,
                              &hoodSubsystem,       &intakeSubsystem,  &conveyorSubsystem};
    MoveForward5Ft moveForwardAuto{&drivetrainSubsystem};
    MoveBackAuto moveBackAuto{&drivetrainSubsystem};

    frc::SlewRateLimiter<units::scalar> speedLimiter{4 / 1_s};
    frc::SlewRateLimiter<units::scalar> rotLimiter{4 / 1_s};

    frc::XboxController m_driverController{str::oi::DRIVER_CONTROLLER_PORT};
    frc::XboxController m_operatorController{str::oi::OPERATOR_CONTROLLER_PORT};
};
