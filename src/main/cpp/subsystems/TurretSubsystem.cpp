// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "str/Units.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/RobotBase.h"
#include <iostream>

TurretSubsystem::TurretSubsystem() {
    ConfigureMotors();
    frc::SmartDashboard::PutData("Turret PID", &controller);
    UnlockTurret();
}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {
    auto currentAngle = GetCurrentTurretAngle();
    frc::TrapezoidProfile<units::radians>::State goal;
    frc::SmartDashboard::PutNumber("Turret Setpoint",
                                   units::convert<units::radian, units::degree>(turretSetpointGoal).to<double>());
    frc::SmartDashboard::PutNumber("Turret Current Angle",
                                   units::convert<units::radian, units::degree>(currentAngle).to<double>());
    if (turretSetpointGoal > 60_deg) {
        turretSetpointGoal = 60_deg;
    }
    if (turretSetpointGoal < -60_deg) {
        turretSetpointGoal = -60_deg;
    }
    controller.SetGoal(turretSetpointGoal);
    double controllerOutput = controller.Calculate(currentAngle);
    if (!homing) {
        turretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, controllerOutput);
    } else {
        if (turretMotor.IsFwdLimitSwitchClosed()) {
            homing = false;
            turretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
        } else {
            turretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, .25);
        }
    }
}

void TurretSubsystem::SimulationPeriodic() {
    turretSim.SetInput(Eigen::Vector<double, 1>{turretMotor.GetMotorOutputVoltage()});
    turretSim.Update(20_ms);
    turretSimCollection.SetIntegratedSensorVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
        turretSim.GetVelocity(), str::encoder_cpr::TALON_FX_ENCODER_CPR, str::physical_dims::TURRET_GEARBOX_RATIO));
    turretSimCollection.SetIntegratedSensorRawPosition(str::Units::ConvertAngleToEncoderTicks(
        turretSim.GetAngle(), str::encoder_cpr::TALON_FX_ENCODER_CPR, str::physical_dims::TURRET_GEARBOX_RATIO, false));
    turretSimCollection.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
    turretArm->SetAngle(turretSim.GetAngle());
}

void TurretSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig;
    baseConfig.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
    baseConfig.forwardLimitSwitchSource =
        ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
    baseConfig.reverseLimitSwitchSource =
        ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_FeedbackConnector;
    ;
    baseConfig.forwardLimitSwitchNormal =
        ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;
    baseConfig.reverseLimitSwitchNormal =
        ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_NormallyOpen;

    turretMotor.ConfigAllSettings(baseConfig);
    turretMotor.SetInverted(false);

    turretMotor.ConfigForwardSoftLimitEnable(true);
    turretMotor.ConfigReverseSoftLimitEnable(true);
    turretMotor.ConfigForwardSoftLimitThreshold(15000);
    turretMotor.ConfigReverseSoftLimitThreshold(-16000);

    turretMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

units::ampere_t TurretSubsystem::GetCurrentDraw() const {
    return turretSim.GetCurrentDraw();
}

void TurretSubsystem::SetTurretGoal(units::radian_t goal) {
    turretSetpointGoal = goal;
}

units::degree_t TurretSubsystem::GetTurretSetpoint() {
    return turretSetpointGoal;
}

units::radian_t TurretSubsystem::GetCurrentTurretAngle() {
    return str::Units::ConvertTicksToAngle(turretMotor.GetSelectedSensorPosition(),
                                           str::encoder_cpr::TALON_FX_ENCODER_CPR,
                                           str::physical_dims::TURRET_GEARBOX_RATIO, false);
}

frc::Transform2d TurretSubsystem::GetCameraToRobotPose() {
    units::radian_t currentTurretAngle = GetCurrentTurretAngle();
    units::inch_t x_coord_turret = 9.05_in * units::math::sin(currentTurretAngle);
    units::inch_t y_coord_turret = 9.05_in * units::math::cos(currentTurretAngle);
    return frc::Transform2d(frc::Translation2d(y_coord_turret + str::vision_vars::CAMERA_TO_ROBOT.Y(),
                                               x_coord_turret + str::vision_vars::CAMERA_TO_ROBOT.X()),
                            frc::Rotation2d(currentTurretAngle));
}

void TurretSubsystem::HomeTurret() {
    homing = true;
}

void TurretSubsystem::LockTurret() {
    turretLockSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void TurretSubsystem::UnlockTurret() {
    turretLockSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

bool TurretSubsystem::IsAtSetpoint() {
    auto positionError = turretSetpointGoal - GetCurrentTurretAngle();
    if (units::math::abs(positionError) < str::turret_pid::TURRET_ALLOWABLE_ERROR) {
        return true;
    } else {
        return false;
    }
}