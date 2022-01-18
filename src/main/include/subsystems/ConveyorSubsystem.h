// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <TimeOfFlight.h>
#include "Constants.h"
#include <frc/filter/MedianFilter.h>
#include "str/MockToF.h"

class ConveyorSubsystem : public frc2::SubsystemBase {
public:
    ConveyorSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    bool DoesTopSensorSeeBall();
    bool DoesBottomSensorSeeBall();
    void SetBottomConveyorSpeed(double speed);
    void SetTopConveyorSpeed(double speed);

private:
    void ConfigureMotors();
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX bottomConveyorMotor{str::can_ids::BOTTOM_CONVEYOR_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX topConveyorMotor{str::can_ids::TOP_CONVEYOR_TALON_ID};
#ifdef __FRC_ROBORIO__
    frc::TimeOfFlight bottomConveyorSensor{str::can_ids::BOTTOM_CONVEYOR_RANGE_SENSOR_ID};
    frc::TimeOfFlight topConveyorSensor{str::can_ids::TOP_CONVEYOR_RANGE_SENSOR_ID};
#else
    MockToF bottomConveyorSensor{str::can_ids::BOTTOM_CONVEYOR_RANGE_SENSOR_ID};
    MockToF topConveyorSensor{str::can_ids::TOP_CONVEYOR_RANGE_SENSOR_ID};
#endif
    frc::MedianFilter<double> bottomFilter{str::intake_vars::FILTER_VALUE};
    frc::MedianFilter<double> topFilter{str::intake_vars::FILTER_VALUE};
    units::millimeter_t bottomDistFiltered = 0_m;
    units::millimeter_t topDistFiltered = 0_m;
};
