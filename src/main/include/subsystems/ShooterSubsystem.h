// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/simulation/FlywheelSim.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include "Constants.h"
#include "str/ContinuousServo.h"
#include <frc/Encoder.h>
#include "subsystems/VisionSubsystem.h"
#include <frc/controller/PIDController.h>
#include "str/ShooterLookupTable.h"

class ShooterSubsystem : public frc2::SubsystemBase {
public:
    ShooterSubsystem(VisionSubsystem* visionSub, str::ShooterLookupTable* shooterTable);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void SimulationPeriodic() override;
    units::radians_per_second_t GetCurrentShooterSpeed();
    units::meters_per_second_t GetWheelSurfaceSpeed(units::radians_per_second_t angularSpeed);
    units::ampere_t GetCurrentDraw() const;
    void SetShooterSpeed(units::revolutions_per_minute_t setSpeed);
    void SetShooterSurfaceSpeed(units::feet_per_second_t setSurfaceSpeed);
    void SetShooterSpeedPercent(double setSpeed);
    const units::radians_per_second_t GetShooterSetpoint() const;
    bool IsFlywheelUpToSpeed();
    units::revolutions_per_minute_t GetShooterSpeedToGoTo();

private:
    VisionSubsystem* visionSubsystem;
    str::ShooterLookupTable* lookupTable;
    void ResetEncoders();
    void ConfigureMotors();
    ctre::phoenix::motorcontrol::can::WPI_TalonFX shooterMotorLeader{str::can_ids::SHOOTERLEADER_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX shooterMotorFollower01{str::can_ids::SHOOTERFOLLOWER01_TALON_ID};
    ctre::phoenix::motorcontrol::TalonFXSimCollection shooterSimCollection{shooterMotorLeader};
    frc::sim::FlywheelSim shooterSim{str::shooter_pid::SHOOTER_PLANT, str::physical_dims::SHOOTER_GEARBOX,
                                     str::physical_dims::SHOOTER_GEARBOX_RATIO};
    frc::PIDController controller{str::shooter_pid::SHOOTER_KP, str::shooter_pid::SHOOTER_KI,
                                  str::shooter_pid::SHOOTER_KD};
    frc::SimpleMotorFeedforward<units::radian> feedforward{str::shooter_pid::KS, str::shooter_pid::KV,
                                                           str::shooter_pid::KA};
    units::radians_per_second_t currentShooterSpeedSetpoint;
    units::revolutions_per_minute_t shooterSpeedToGoTo;
};
