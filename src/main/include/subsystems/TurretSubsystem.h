// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include "Constants.h"
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <wpi/numbers>
#include <frc/system/LinearSystemLoop.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/SimpleMotorFeedforward.h>

class TurretSubsystem : public frc2::SubsystemBase {
public:
    TurretSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void SimulationPeriodic() override;
    units::ampere_t GetCurrentDraw() const;
    void SetTurretGoal(units::radian_t goal);
    units::degree_t GetTurretSetpoint();
    frc::Transform2d GetCameraToRobotPose();
    units::radian_t GetCurrentTurretAngle();
    void HomeTurret();
    void LockTurret();
    void UnlockTurret();
    bool IsAtSetpoint();

private:
    void ConfigureMotors();
    ctre::phoenix::motorcontrol::can::TalonFX turretMotor{str::can_ids::TURRET_TALON_ID};
    ctre::phoenix::motorcontrol::TalonFXSimCollection turretSimCollection{turretMotor};
    frc::TrapezoidProfile<units::radians>::Constraints constraints{70_deg_per_s, 70_deg_per_s / 1_s};
    frc::TrapezoidProfile<units::radians>::State lastProfiledReference;
    frc::KalmanFilter<2, 1, 1> observer{str::turret_pid::TURRET_PLANT, {0.015, 0.17}, {0.01}, 20_ms};
    frc::LinearQuadraticRegulator<2, 1> controller{str::turret_pid::TURRET_PLANT,
                                                   // qelms. Velocity error tolerance, in radians and radians per
                                                   // second. Decrease this to more heavily penalize state excursion, or
                                                   // make the controller behave more aggressively.
                                                   {.5, 1.0},
                                                   // relms. Control effort (voltage) tolerance. Decrease this to more
                                                   // heavily penalize control effort, or make the controller less
                                                   // aggressive. 12 is a good starting point because that is the
                                                   // (approximate) maximum voltage of a battery.
                                                   {3},
                                                   20_ms};
    frc::LinearSystemLoop<2, 1, 1> loop{str::turret_pid::TURRET_PLANT, controller, observer, 12_V, 20_ms};
    frc::sim::SingleJointedArmSim turretSim{str::physical_dims::TURRET_GEARBOX,
                                            str::physical_dims::TURRET_GEARBOX_RATIO,
                                            str::physical_dims::TURRET_MOI,
                                            str::physical_dims::TURRET_DIAMETER,
                                            -120_deg,
                                            120_deg,
                                            str::physical_dims::TURRET_MASS,
                                            false};
    frc::Mechanism2d turretViz{60, 60};
    frc::MechanismRoot2d* turretBase = turretViz.GetRoot("TurretBase", 30, 30);
    frc::MechanismLigament2d* turretArm = turretBase->Append<frc::MechanismLigament2d>(
        "TurretArm", 30, turretSim.GetAngle(), 6, frc::Color8Bit{frc::Color::kCyan});
    units::radian_t turretSetpointGoal = 0_deg;
    bool homing = false;
    frc::DoubleSolenoid turretLockSolenoid{frc::PneumaticsModuleType::CTREPCM, str::pcm_ports::TURRET_LOCK_PORT1,
                                           str::pcm_ports::TURRET_LOCK_PORT2};
    frc::SimpleMotorFeedforward<units::radian> feedforward{str::turret_pid::KS, str::turret_pid::KV,
                                                           str::turret_pid::KA};
};
