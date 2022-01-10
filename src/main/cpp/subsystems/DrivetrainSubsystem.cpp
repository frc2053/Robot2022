// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "str/Units.h"
#include "subsystems/DrivetrainSubsystem.h"
#include <iostream>
#include "str/ExampleVisionSensor.h"

DrivetrainSubsystem::DrivetrainSubsystem() {
    gyro.Calibrate();
    gyro.ZeroYaw();

    ConfigureMotors();
    ResetEncoders();
    frc::SmartDashboard::PutData("Field", &fieldSim);
    frc::SmartDashboard::PutData("Pose Estimator Field", &poseEstimatorSim);
}

void DrivetrainSubsystem::Periodic() {
    frc::Rotation2d currentGyroYaw = gyro.GetYaw();
    units::meter_t leftEncoderDistance =
        str::Units::ConvertEncoderTicksToDistance(
            frontLeftTalon.GetSelectedSensorPosition(),
            str::encoder_cpr::TALON_FX_ENCODER_CPR,
            str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
            str::physical_dims::WHEEL_DIAMETER / 2);
    units::meter_t rightEncoderDistance =
        str::Units::ConvertEncoderTicksToDistance(
            frontRightTalon.GetSelectedSensorPosition(),
            str::encoder_cpr::TALON_FX_ENCODER_CPR,
            str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
            str::physical_dims::WHEEL_DIAMETER / 2);
    units::meters_per_second_t leftEncoderVelocity =
        str::Units::ConvertAngularVelocityToLinearVelocity(
            str::Units::ConvertTicksPer100MsToAngularVelocity(
                frontLeftTalon.GetSelectedSensorVelocity(),
                str::encoder_cpr::TALON_FX_ENCODER_CPR,
                str::physical_dims::DRIVEBASE_GEARBOX_RATIO),
            str::physical_dims::WHEEL_DIAMETER / 2);
    units::meters_per_second_t rightEncoderVelocity =
        str::Units::ConvertAngularVelocityToLinearVelocity(
            str::Units::ConvertTicksPer100MsToAngularVelocity(
                frontRightTalon.GetSelectedSensorVelocity(),
                str::encoder_cpr::TALON_FX_ENCODER_CPR,
                str::physical_dims::DRIVEBASE_GEARBOX_RATIO),
            str::physical_dims::WHEEL_DIAMETER / 2);

    poseEstimator.Update(currentGyroYaw,
                         {leftEncoderVelocity, rightEncoderVelocity},
                         leftEncoderDistance, rightEncoderDistance);

    odom.Update(currentGyroYaw, leftEncoderDistance, rightEncoderDistance);

    fieldSim.SetRobotPose(odom.GetPose());
    poseEstimatorSim.SetRobotPose(poseEstimator.GetEstimatedPosition());
}

void DrivetrainSubsystem::SimulationPeriodic() {
    drivetrainSimulator.SetInputs(
        units::volt_t{frontLeftTalon.GetMotorOutputVoltage()},
        units::volt_t{frontRightTalon.GetMotorOutputVoltage()});
    drivetrainSimulator.Update(20_ms);

    leftSimCollection.SetIntegratedSensorRawPosition(
        str::Units::ConvertDistanceToEncoderTicks(
            drivetrainSimulator.GetLeftPosition(),
            str::encoder_cpr::TALON_FX_ENCODER_CPR,
            str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
            str::physical_dims::WHEEL_DIAMETER / 2));
    leftSimCollection.SetIntegratedSensorVelocity(
        str::Units::ConvertAngularVelocityToTicksPer100Ms(
            str::Units::ConvertLinearVelocityToAngularVelocity(
                drivetrainSimulator.GetLeftVelocity(),
                str::physical_dims::WHEEL_DIAMETER / 2),
            str::encoder_cpr::TALON_FX_ENCODER_CPR,
            str::physical_dims::DRIVEBASE_GEARBOX_RATIO));
    rightSimCollection.SetIntegratedSensorRawPosition(
        str::Units::ConvertDistanceToEncoderTicks(
            drivetrainSimulator.GetRightPosition(),
            str::encoder_cpr::TALON_FX_ENCODER_CPR,
            str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
            str::physical_dims::WHEEL_DIAMETER / 2));
    rightSimCollection.SetIntegratedSensorVelocity(
        str::Units::ConvertAngularVelocityToTicksPer100Ms(
            str::Units::ConvertLinearVelocityToAngularVelocity(
                drivetrainSimulator.GetRightVelocity(),
                str::physical_dims::WHEEL_DIAMETER / 2),
            str::encoder_cpr::TALON_FX_ENCODER_CPR,
            str::physical_dims::DRIVEBASE_GEARBOX_RATIO));

    gyro.SetYaw(drivetrainSimulator.GetHeading().Degrees().to<double>());

    leftSimCollection.SetBusVoltage(
        frc::RobotController::GetBatteryVoltage().to<double>());
    rightSimCollection.SetBusVoltage(
        frc::RobotController::GetBatteryVoltage().to<double>());
}

void DrivetrainSubsystem::ArcadeDrive(double fwd, double rot) {
    drive.ArcadeDrive(fwd, rot, false);
}

void DrivetrainSubsystem::CurvatureDrive(double fwd, double rot,
                                         bool quickTurn) {
    drive.CurvatureDrive(fwd, rot, quickTurn);
}

void DrivetrainSubsystem::TankDriveVolts(units::volt_t left,
                                         units::volt_t right) {
    frontLeftTalon.SetVoltage(left);
    frontRightTalon.SetVoltage(right);
    drive.Feed();
}

units::ampere_t DrivetrainSubsystem::GetCurrentDraw() const {
    return drivetrainSimulator.GetCurrentDraw();
}

units::degree_t DrivetrainSubsystem::GetHeading() {
    return gyro.GetYaw().Degrees();
}

frc::Pose2d DrivetrainSubsystem::GetPose() {
    return odom.GetPose();
}

units::degrees_per_second_t DrivetrainSubsystem::GetTurnRate() {
    return gyro.GetYawRate();
}

void DrivetrainSubsystem::AddVisionMeasurement(frc::Pose2d visionPose,
                                               units::second_t latency) {
    poseEstimator.AddVisionMeasurement(
        visionPose, frc::Timer::GetFPGATimestamp() - latency);
}

frc::DifferentialDriveWheelSpeeds DrivetrainSubsystem::GetWheelSpeeds() {
    return {str::Units::ConvertAngularVelocityToLinearVelocity(
                str::Units::ConvertTicksPer100MsToAngularVelocity(
                    frontLeftTalon.GetSelectedSensorVelocity(),
                    str::encoder_cpr::TALON_FX_ENCODER_CPR,
                    str::physical_dims::DRIVEBASE_GEARBOX_RATIO),
                str::physical_dims::WHEEL_DIAMETER / 2),
            str::Units::ConvertAngularVelocityToLinearVelocity(
                str::Units::ConvertTicksPer100MsToAngularVelocity(
                    frontRightTalon.GetSelectedSensorVelocity(),
                    str::encoder_cpr::TALON_FX_ENCODER_CPR,
                    str::physical_dims::DRIVEBASE_GEARBOX_RATIO),
                str::physical_dims::WHEEL_DIAMETER / 2)};
}

void DrivetrainSubsystem::DrawTrajectory(frc::Trajectory traj) {
    fieldSim.GetObject("traj" + std::to_string(trajCounter))
        ->SetTrajectory(traj);
    trajCounter++;
}

void DrivetrainSubsystem::ResetOdom(frc::Pose2d pose) {
    ResetEncoders();
    drivetrainSimulator.SetPose(pose);
    odom.ResetPosition(pose, gyro.GetYaw());
    std::cout << "Reset Odom on drivetrain!\n";
}

void DrivetrainSubsystem::SetGyroOffset(units::degree_t offset) {
    gyro.SetOffset(offset);
}

void DrivetrainSubsystem::ConfigureMotors() {
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
    baseConfig.slot0.kF = str::drive_pid::KF;
    baseConfig.slot0.kP = str::drive_pid::KP;
    baseConfig.slot0.kI = str::drive_pid::KI;
    baseConfig.slot0.kD = str::drive_pid::KD;

    frontLeftTalon.ConfigAllSettings(baseConfig);
    rearLeftTalon.ConfigAllSettings(baseConfig);

    frontRightTalon.ConfigAllSettings(baseConfig);
    rearRightTalon.ConfigAllSettings(baseConfig);

    rearLeftTalon.Follow(frontLeftTalon);
    rearLeftTalon.SetInverted(
        ctre::phoenix::motorcontrol::InvertType::FollowMaster);

    rearRightTalon.Follow(frontRightTalon);
    rearRightTalon.SetInverted(
        ctre::phoenix::motorcontrol::InvertType::FollowMaster);

    if (frc::RobotBase::IsSimulation()) {
        frontLeftTalon.SetInverted(
            ctre::phoenix::motorcontrol::InvertType::None);
        frontRightTalon.SetInverted(
            ctre::phoenix::motorcontrol::InvertType::None);
    } else {
        frontLeftTalon.SetInverted(
            ctre::phoenix::motorcontrol::InvertType::InvertMotorOutput);
        frontRightTalon.SetInverted(
            ctre::phoenix::motorcontrol::InvertType::None);
    }
}

void DrivetrainSubsystem::ResetEncoders() {
    leftSimCollection.SetIntegratedSensorRawPosition(0);
    rightSimCollection.SetIntegratedSensorRawPosition(0);
    frontLeftTalon.SetSelectedSensorPosition(0);
    rearLeftTalon.SetSelectedSensorPosition(0);
    frontRightTalon.SetSelectedSensorPosition(0);
    rearRightTalon.SetSelectedSensorPosition(0);
}
