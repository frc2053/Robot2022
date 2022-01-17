// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <TimeOfFlight.h>
#include "Constants.h"

class ConveyorSubsystem : public frc2::SubsystemBase {
public:
    ConveyorSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

private:
    void ConfigureMotors();
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX bottomConveyorMotor{str::can_ids::BOTTOM_CONVEYOR_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX topConveyorMotor{str::can_ids::TOP_CONVEYOR_TALON_ID};
    frc::TimeOfFlight bottomConveyorSensor{str::can_ids::BOTTOM_CONVEYOR_RANGE_SENSOR_ID};
    frc::TimeOfFlight topConveyorSensor{str::can_ids::TOP_CONVEYOR_RANGE_SENSOR_ID};
    int numOfBalls = 0;
};
