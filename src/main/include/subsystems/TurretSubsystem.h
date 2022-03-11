// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <frc/simulation/SingleJointedArmSim.h>
#include <wpi/numbers>
#include <frc/system/LinearSystemLoop.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/ProfiledPIDController.h>

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

private:
    void ConfigureMotors();
    ctre::phoenix::motorcontrol::can::TalonFX turretMotor{str::can_ids::TURRET_TALON_ID};
    ctre::phoenix::motorcontrol::TalonFXSimCollection turretSimCollection{turretMotor};
    frc::TrapezoidProfile<units::radians>::Constraints constraints{360_deg_per_s, 3600_deg_per_s / 1_s};
    frc::ProfiledPIDController<units::radians> controller{str::turret_pid::KP, str::turret_pid::KI, str::turret_pid::KD,
                                                          constraints, 20_ms};
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
};
