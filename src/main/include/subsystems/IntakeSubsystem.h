// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/DoubleSolenoid.h>
#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void PutIntakeDown();
    void PutIntakeUp();

private:
    void ConfigureMotors();
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX intakeMotor{str::can_ids::INTAKE_TALON_ID};
    frc::DoubleSolenoid intakeSolenoid{frc::PneumaticsModuleType::CTREPCM, str::pcm_ports::INTAKE_SOLENOID_PORT1,
                                       str::pcm_ports::INTAKE_SOLENOID_PORT2};
};
