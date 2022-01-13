// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "str/Units.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem() {
    loop.Reset(Eigen::Vector<double, 1>{});
    frc::SmartDashboard::PutNumber("Shooter Set Speed", 0);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    loop.SetNextR(Eigen::Vector<double, 1>{currentShooterSpeedSetpoint.value()});
    units::radians_per_second_t wheelAngularSpeed = GetCurrentShooterSpeed();
    units::meters_per_second_t wheelSurfaceSpeed = GetWheelSurfaceSpeed(wheelAngularSpeed);
    frc::SmartDashboard::PutNumber("Shooter Motor Angular Speed", units::convert<units::rad_per_s, units::rpm>(wheelAngularSpeed).to<double>());
    frc::SmartDashboard::PutNumber("Shooter Motor Surface Speed", units::convert<units::meters_per_second, units::feet_per_second>(wheelSurfaceSpeed).to<double>());
    loop.Correct(Eigen::Vector<double, 1>{wheelAngularSpeed.value()});
    loop.Predict(20_ms);
    auto finalVoltage = units::volt_t(loop.U(0)) + feedforward.Calculate(currentShooterSpeedSetpoint);
    shooterMotorLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, finalVoltage / 12_V);
}

void ShooterSubsystem::SimulationPeriodic() {
    shooterSim.SetInput(Eigen::Vector<double, 1>{shooterMotorLeader.GetMotorOutputVoltage()});
    shooterSim.Update(20_ms);
    int simTickVelocity = str::Units::ConvertAngularVelocityToTicksPer100Ms(
        shooterSim.GetAngularVelocity(), 
        str::encoder_cpr::TALON_FX_ENCODER_CPR, 
        str::physical_dims::SHOOTER_GEARBOX_RATIO
    );
    shooterSimCollection.SetIntegratedSensorVelocity(simTickVelocity);
    shooterSimCollection.SetStatorCurrent(shooterSim.GetCurrentDraw().to<double>());
    shooterSimCollection.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
}

void ShooterSubsystem::SetShooterSpeed(units::revolutions_per_minute_t setSpeed) {
   currentShooterSpeedSetpoint = setSpeed;
}

void ShooterSubsystem::SetShooterSurfaceSpeed(units::feet_per_second_t setSurfaceSpeed) {
    currentShooterSpeedSetpoint = str::Units::ConvertLinearVelocityToAngularVelocity(setSurfaceSpeed, str::physical_dims::SHOOTER_WHEEL_DIAMETER / 2);
}

const units::radians_per_second_t ShooterSubsystem::GetShooterSetpoint() const {
    return currentShooterSpeedSetpoint;
}

units::radians_per_second_t ShooterSubsystem::GetCurrentShooterSpeed() {
    auto retVal = str::Units::ConvertTicksPer100MsToAngularVelocity(
        shooterMotorLeader.GetSelectedSensorVelocity(), 
        str::encoder_cpr::TALON_FX_ENCODER_CPR, 
        str::physical_dims::SHOOTER_GEARBOX_RATIO
    );
    return retVal;
}

units::meters_per_second_t ShooterSubsystem::GetWheelSurfaceSpeed(units::radians_per_second_t angularSpeed) {
    auto retVal = str::Units::ConvertAngularVelocityToLinearVelocity(angularSpeed, str::physical_dims::SHOOTER_WHEEL_DIAMETER / 2);
    return retVal;
}

units::ampere_t ShooterSubsystem::GetCurrentDraw() const {
    return shooterSim.GetCurrentDraw();
}

void ShooterSubsystem::ResetEncoders() {
    shooterSimCollection.SetIntegratedSensorRawPosition(0);
    shooterSimCollection.SetIntegratedSensorVelocity(0);
    shooterMotorLeader.SetSelectedSensorPosition(0);
    shooterMotorFollower01.SetSelectedSensorPosition(0);
    shooterMotorFollower02.SetSelectedSensorPosition(0);
}

void ShooterSubsystem::ConfigureMotors() {
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig;
    baseConfig.primaryPID.selectedFeedbackSensor =
        ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
    baseConfig.forwardLimitSwitchSource =
        ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource =
        ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal =
        ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal =
        ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    shooterMotorLeader.ConfigAllSettings(baseConfig);
    shooterMotorFollower01.ConfigAllSettings(baseConfig);
    shooterMotorFollower02.ConfigAllSettings(baseConfig);

    shooterMotorFollower01.Follow(shooterMotorLeader);
    shooterMotorFollower01.SetInverted(
        ctre::phoenix::motorcontrol::InvertType::FollowMaster);

    shooterMotorFollower02.Follow(shooterMotorLeader);
    shooterMotorFollower02.SetInverted(
        ctre::phoenix::motorcontrol::InvertType::FollowMaster);
}
