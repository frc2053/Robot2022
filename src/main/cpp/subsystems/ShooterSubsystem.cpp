// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"
#include "str/Units.h"
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem(VisionSubsystem* visionSub) : visionSubsystem(visionSub) {
    SetName("ShooterSubsystem");
    loop.Reset(Eigen::Vector<double, 1>{});
    ConfigureMotors();
    hoodEncoder.SetSamplesToAverage(50);
    hoodEncoder.SetMinRate(1.0);
    hoodEncoder.SetDistancePerPulse(1);
    hoodEncoder.SetReverseDirection(true);
    hoodController.SetSetpoint(0);
    hoodController.SetTolerance(str::shooter_pid::HOOD_TOLERANCE);
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
    str::LookupValue goalTarget = lookupTable.Get(visionSubsystem->GetDistanceToTarget());
    shooterSpeedToGoTo = goalTarget.rpm;
    hoodAngleToGoTo = goalTarget.angle;

    double hoodOutputVal = hoodController.Calculate(GetHoodAngle().to<double>());
    SetServoSpeed(hoodOutputVal);

    loop.SetNextR(Eigen::Vector<double, 1>{currentShooterSpeedSetpoint.value()});
    units::radians_per_second_t wheelAngularSpeed = GetCurrentShooterSpeed();
    units::meters_per_second_t wheelSurfaceSpeed = GetWheelSurfaceSpeed(wheelAngularSpeed);
    frc::SmartDashboard::PutNumber("Shooter Motor Angular Speed",
                                   units::convert<units::rad_per_s, units::rpm>(wheelAngularSpeed).to<double>());
    frc::SmartDashboard::PutNumber(
        "Shooter Motor Surface Speed",
        units::convert<units::meters_per_second, units::feet_per_second>(wheelSurfaceSpeed).to<double>());
    frc::SmartDashboard::PutNumber(
        "Shooter Motor Setpoint",
        units::convert<units::rad_per_s, units::rpm>(currentShooterSpeedSetpoint).to<double>());
    loop.Correct(Eigen::Vector<double, 1>{wheelAngularSpeed.value()});
    loop.Predict(20_ms);
    auto finalVoltage = units::volt_t(loop.U(0)) + feedforward.Calculate(currentShooterSpeedSetpoint);
    shooterMotorLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, finalVoltage / 12_V);
}

void ShooterSubsystem::SimulationPeriodic() {
    shooterSim.SetInput(Eigen::Vector<double, 1>{shooterMotorLeader.GetMotorOutputVoltage()});
    shooterSim.Update(20_ms);
    int simTickVelocity = str::Units::ConvertAngularVelocityToTicksPer100Ms(shooterSim.GetAngularVelocity(),
                                                                            str::encoder_cpr::TALON_FX_ENCODER_CPR,
                                                                            str::physical_dims::SHOOTER_GEARBOX_RATIO);
    shooterSimCollection.SetIntegratedSensorVelocity(simTickVelocity);
    shooterSimCollection.SetStatorCurrent(shooterSim.GetCurrentDraw().to<double>());
    shooterSimCollection.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
}

void ShooterSubsystem::SetShooterSpeed(units::revolutions_per_minute_t setSpeed) {
    currentShooterSpeedSetpoint = setSpeed;
}

void ShooterSubsystem::SetShooterSpeedPercent(double setSpeed) {
    shooterMotorLeader.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, setSpeed);
}

void ShooterSubsystem::SetShooterSurfaceSpeed(units::feet_per_second_t setSurfaceSpeed) {
    currentShooterSpeedSetpoint = str::Units::ConvertLinearVelocityToAngularVelocity(
        setSurfaceSpeed, str::physical_dims::SHOOTER_WHEEL_DIAMETER / 2);
}

const units::radians_per_second_t ShooterSubsystem::GetShooterSetpoint() const {
    return currentShooterSpeedSetpoint;
}

units::radians_per_second_t ShooterSubsystem::GetCurrentShooterSpeed() {
    auto retVal = str::Units::ConvertTicksPer100MsToAngularVelocity(shooterMotorLeader.GetSelectedSensorVelocity(),
                                                                    str::encoder_cpr::TALON_FX_ENCODER_CPR,
                                                                    str::physical_dims::SHOOTER_GEARBOX_RATIO);
    return retVal;
}

units::meters_per_second_t ShooterSubsystem::GetWheelSurfaceSpeed(units::radians_per_second_t angularSpeed) {
    auto retVal = str::Units::ConvertAngularVelocityToLinearVelocity(angularSpeed,
                                                                     str::physical_dims::SHOOTER_WHEEL_DIAMETER / 2);
    return retVal;
}

units::ampere_t ShooterSubsystem::GetCurrentDraw() const {
    return shooterSim.GetCurrentDraw();
}

str::LookupValue ShooterSubsystem::GetAngleAndRPMForGoal(units::meter_t distance) {
    return lookupTable.Get(distance);
}

bool ShooterSubsystem::IsFlywheelUpToSpeed() {
    return units::math::abs(GetCurrentShooterSpeed() - currentShooterSpeedSetpoint) <
           str::shooter_pid::FLYWHEEL_ALLOWABLE_ERROR;
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
    baseConfig.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
    baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
    baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
    baseConfig.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
    baseConfig.velocityMeasurementWindow = 1;
    shooterMotorLeader.ConfigAllSettings(baseConfig);
    shooterMotorFollower01.ConfigAllSettings(baseConfig);
    shooterMotorFollower02.ConfigAllSettings(baseConfig);

    shooterMotorLeader.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    shooterMotorLeader.SetInverted(false);
    shooterMotorLeader.ConfigPeakOutputReverse(0);

    shooterMotorFollower01.Follow(shooterMotorLeader);
    shooterMotorFollower01.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);

    shooterMotorFollower02.Follow(shooterMotorLeader);
    shooterMotorFollower02.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
}

int ShooterSubsystem::ConvertHoodAngleToTicks(units::degree_t angle) {
    return str::Units::map(angle.to<double>(), 0, str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value(), 0,
                           str::shooter_pid::SHOOTER_HOOD_MAX_TICKS);
}

units::degree_t ShooterSubsystem::ConvertHoodTicksToAngle(int ticks) {
    return units::degree_t(str::Units::map(ticks, 0, str::shooter_pid::SHOOTER_HOOD_MAX_TICKS, 0,
                                           str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value()));
}

units::degree_t ShooterSubsystem::GetHoodAngle() {
    return ConvertHoodTicksToAngle(hoodEncoder.Get());
}

void ShooterSubsystem::SetHoodToAngle(units::degree_t setpoint) {
    hoodController.SetSetpoint(setpoint.to<double>());
}

void ShooterSubsystem::SetServoSpeed(double percent) {
    hoodServo.Set(std::clamp(percent, -1.0, 1.0));
}

units::degree_t ShooterSubsystem::GetHoodAngleToGoTo() {
    return hoodAngleToGoTo;
}

units::revolutions_per_minute_t ShooterSubsystem::GetShooterSpeedToGoTo() {
    return shooterSpeedToGoTo;
}

bool ShooterSubsystem::IsHoodAtSetpoint() {
    return hoodController.AtSetpoint();
}