// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "str/Units.h"
#include "frc/smartdashboard/Field2d.h"

TurretSubsystem::TurretSubsystem() {
    ConfigureMotors();
    frc::SmartDashboard::PutData("Turret Sim", &turretViz);
    loop.Reset(Eigen::Vector<double, 2>{});
    lastProfiledReference = {};
    UnlockTurret();
}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {
    auto currentAngle = GetCurrentTurretAngle();
    frc::TrapezoidProfile<units::radians>::State goal;
    goal = {turretSetpointGoal, 0_rad_per_s};
    frc::SmartDashboard::PutNumber("Turret Setpoint",
                                   units::convert<units::radian, units::degree>(turretSetpointGoal).to<double>());
    frc::SmartDashboard::PutNumber("Turret Current Angle",
                                   units::convert<units::radian, units::degree>(currentAngle).to<double>());
    lastProfiledReference =
        (frc::TrapezoidProfile<units::radians>(constraints, goal, lastProfiledReference)).Calculate(20_ms);

    if (!homing) {
        loop.SetNextR(
            Eigen::Vector<double, 2>{lastProfiledReference.position.value(), lastProfiledReference.velocity.value()});
        loop.Correct(Eigen::Vector<double, 1>{currentAngle.value()});
        loop.Predict(20_ms);
        turretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, units::volt_t(loop.U(0)) / 12_V);
    } else {
        if (turretMotor.IsRevLimitSwitchClosed()) {
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
    turretSimCollection.SetQuadratureVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
        turretSim.GetVelocity(), str::encoder_cpr::CANCODER_ENCODER_CPR, str::physical_dims::TURRET_GEARBOX_RATIO));
    turretSimCollection.SetQuadratureRawPosition(str::Units::ConvertAngleToEncoderTicks(
        turretSim.GetAngle(), str::encoder_cpr::CANCODER_ENCODER_CPR, str::physical_dims::TURRET_GEARBOX_RATIO, true));
    turretSimCollection.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
    turretArm->SetAngle(turretSim.GetAngle());
}

void TurretSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonSRXConfiguration baseConfig;
    baseConfig.primaryPID.selectedFeedbackSensor =
        ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative;
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

    turretMotor.ConfigClearPositionOnLimitR(true, 0);
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
                                           str::encoder_cpr::CANCODER_ENCODER_CPR,
                                           str::physical_dims::TURRET_GEARBOX_RATIO, true);
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