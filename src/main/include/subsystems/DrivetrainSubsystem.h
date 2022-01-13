// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/SubsystemBase.h>
#include "patch/DifferentialDrivePoseEstimatorPATCH.h"
//#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "Constants.h"
#include "str/IMU.h"
#include <frc/smartdashboard/Field2d.h>

class DrivetrainSubsystem : public frc2::SubsystemBase {
   public:
    DrivetrainSubsystem();
    void Periodic() override;
    void SimulationPeriodic() override;

    void ArcadeDrive(double fwd, double rot);
    void CurvatureDrive(double fwd, double rot, bool quickTurn);
    void TankDriveVolts(units::volt_t left, units::volt_t right);
    void TankDriveVelocity(units::meters_per_second_t left, units::meters_per_second_t right, units::volt_t leftFF, units::volt_t rightFF);

    units::ampere_t GetCurrentDraw() const;
    units::degree_t GetHeading();
    frc::Pose2d GetPose();
    units::degrees_per_second_t GetTurnRate();
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    void DrawTrajectory(frc::Trajectory traj);
    void ResetOdom(frc::Pose2d pose);
    void SetGyroOffset(units::degree_t offset);

    void AddVisionMeasurement(frc::Pose2d visionPose, units::second_t latency);
   private:
    void ConfigureMotors();
    void ResetEncoders();

    str::IMU gyro;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontLeftTalon{
        str::can_ids::FRONT_LEFT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearLeftTalon{
        str::can_ids::REAR_LEFT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontRightTalon{
        str::can_ids::FRONT_RIGHT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearRightTalon{
        str::can_ids::REAR_RIGHT_DRIVEBASE_TALON_ID};

    ctre::phoenix::motorcontrol::TalonFXSimCollection leftSimCollection{
        frontLeftTalon};
    ctre::phoenix::motorcontrol::TalonFXSimCollection rightSimCollection{
        frontRightTalon};
    frc::sim::DifferentialDrivetrainSim drivetrainSimulator{
        str::drive_pid::DRIVE_TRAIN_PLANT,
        str::physical_dims::TRACK_WIDTH,
        str::physical_dims::DRIVEBASE_GEARBOX,
        str::physical_dims::DRIVEBASE_GEARBOX_RATIO,
        str::physical_dims::DRIVE_WHEEL_DIAMETER / 2,
        {0.001, 0.001, 0.0001, 0.1, 0.1, 0.005, 0.005}};

    frc::DifferentialDrive drive{frontLeftTalon, frontRightTalon};
    frc::DifferentialDriveOdometry odom{gyro.GetYaw()};
    frc::DifferentialDrivePoseEstimatorPATCH poseEstimator{
        gyro.GetYaw(),
        frc::Pose2d(),
        {0.01, 0.01, 0.01, 0.01, 0.01},
        {0.1, 0.1, 0.1},
        {0.1, 0.1, 0.1}};
    frc::Field2d fieldSim;
    frc::Field2d poseEstimatorSim;
    int trajCounter = 0;
};
