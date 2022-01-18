// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "str/Units.h"

TurretSubsystem::TurretSubsystem() {
    ConfigureMotors();
    frc::SmartDashboard::PutData("Turret Sim", &turretViz);
    loop.Reset(Eigen::Vector<double, 2>{});
    lastProfiledReference = {};
}

// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {
    frc::TrapezoidProfile<units::radians>::State goal;
    goal = {turretSetpointGoal, 0_rad_per_s};
    lastProfiledReference =
        (frc::TrapezoidProfile<units::radians>(constraints, goal, lastProfiledReference)).Calculate(20_ms);
    loop.SetNextR(
        Eigen::Vector<double, 2>{lastProfiledReference.position.value(), lastProfiledReference.velocity.value()});
    loop.Correct(Eigen::Vector<double, 1>{
        str::Units::ConvertTicksToAngle(turretMotor.GetSelectedSensorPosition(), str::encoder_cpr::CANCODER_ENCODER_CPR,
                                        str::physical_dims::TURRET_GEARBOX_RATIO, true)
            .value()});
    loop.Predict(20_ms);
    turretMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, units::volt_t(loop.U(0)) / 12_V);
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
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    turretMotor.ConfigAllSettings(baseConfig);
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