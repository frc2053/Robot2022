// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include "str/ContinuousServo.h"
#include "Constants.h"
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include "subsystems/VisionSubsystem.h"
#include <frc/Timer.h>
#include "str/ShooterLookupTable.h"

class HoodSubsystem : public frc2::SubsystemBase {
public:
    HoodSubsystem(str::ShooterLookupTable* shooterTable, VisionSubsystem* visionSub);
    void Periodic() override;
    units::degree_t GetHoodAngleToGoTo();
    units::degree_t GetHoodAngle();
    void SetHoodToAngle(units::degree_t setpoint);
    bool IsHoodAtSetpoint();
    str::ShooterLookupTable* lookupTable;

private:
    VisionSubsystem* visionSubsystem;
    int ConvertHoodAngleToTicks(units::degree_t angle);
    units::degree_t ConvertHoodTicksToAngle(int ticks);
    void SetServoSpeed(double percent);
    ContinuousServo hoodServo{str::pwm_ports::HOOD_SERVO_PORT};
    frc::Encoder hoodEncoder{str::dio_ports::HOOD_ENCODER_PORT_A, str::dio_ports::HOOD_ENCODER_PORT_B, false,
                             frc::Encoder::EncodingType::k4X};
    units::degree_t hoodAngleToGoTo;
    frc::PIDController hoodController{str::shooter_pid::HOOD_KP, str::shooter_pid::HOOD_KI, str::shooter_pid::HOOD_KD};
    units::degree_t hoodOffset{str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE};
    bool stallBackwards{false};
    bool stallForwards{false};
    frc::Timer stallTimerF;
    frc::Timer stallTimerB;
    bool startedTimerF{false};
    bool startedTimerB{false};
};
