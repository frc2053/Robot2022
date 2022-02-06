// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/ConveyorSubsystem.h"
#include <iostream>

ConveyorSubsystem::ConveyorSubsystem() {
    SetName("ConveyorSubsystem");
    ConfigureMotors();
    bottomConveyorSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
    topConveyorSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
    frc::SmartDashboard::PutData("Top Conveyor Sensor", &topConveyorSensor);
    frc::SmartDashboard::PutData("Bottom Conveyor Sensor", &bottomConveyorSensor);
    std::cout << std::boolalpha;
}

// This method will be called once per scheduler run
void ConveyorSubsystem::Periodic() {
    double rawBottomDist = bottomConveyorSensor.GetRange();
    double rawTopDist = topConveyorSensor.GetRange();
    // bottomDistFiltered = units::millimeter_t(bottomFilter.Calculate(rawBottomDist));
    // topDistFiltered = units::millimeter_t(topFilter.Calculate(rawTopDist));
    frc::SmartDashboard::PutNumber("top distance raw", rawTopDist);
    frc::SmartDashboard::PutNumber("bottom distance raw", rawBottomDist);
    frc::SmartDashboard::PutNumber("top distance filtered", topDistFiltered.value());
    frc::SmartDashboard::PutNumber("bottom distance filtered", bottomDistFiltered.value());
}

void ConveyorSubsystem::SetFunnelSpeed(double speed) {
    funnelMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

void ConveyorSubsystem::SetConveyorSpeed(double speed) {
    conveyorMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
}

bool ConveyorSubsystem::DoesTopSensorSeeBall() {
    double rawVal = topConveyorSensor.GetRange();
    bool retVal = rawVal < 5.0;
    std::cout << "Does top sensor see ball: " << retVal << "\n";
    return retVal;
}

bool ConveyorSubsystem::DoesBottomSensorSeeBall() {
    double rawVal = bottomConveyorSensor.GetRange();
    bool retVal = rawVal < 5.0;
    std::cout << "Does bottom sensor see ball: " << retVal << "\n";
    return retVal;
}

void ConveyorSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonSRXConfiguration baseConfig;
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    funnelMotor.ConfigAllSettings(baseConfig);
    conveyorMotor.ConfigAllSettings(baseConfig);
    conveyorMotor.SetInverted(true);
}