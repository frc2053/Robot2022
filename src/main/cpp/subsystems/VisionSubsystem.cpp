// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>

VisionSubsystem::VisionSubsystem(DrivetrainSubsystem* driveSub)
    : driveSubsystem(driveSub) {
        gloworm_sim.AddSimVisionTarget(photonlib::SimVisionTarget(
            str::vision_vars::TARGET_POSE,
            str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND,
            str::vision_vars::TARGET_WIDTH,
            str::vision_vars::TARGET_HEIGHT
        ));
    }

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    latestData = gloworm.GetLatestResult();
    if (latestData.HasTargets()) {
        targetsFound = latestData.GetTargets();
        bestTarget = latestData.GetBestTarget();
        auto test = GetTranslationToTarget();
        latency = latestData.GetLatency();
        driveSubsystem->AddVisionMeasurement(GetRobotPose(), latency);
    }
}

void VisionSubsystem::SimulationPeriodic() {
    auto dt_pose = driveSubsystem->GetPose();
    frc::SmartDashboard::PutNumber("DT Pose X", dt_pose.X().to<double>());
    frc::SmartDashboard::PutNumber("DT Pose Y", dt_pose.Y().to<double>());
    frc::SmartDashboard::PutNumber("DT Pose Rot", dt_pose.Rotation().Degrees().to<double>());
    gloworm_sim.ProcessFrame(dt_pose);
}

void VisionSubsystem::ChangeVisionPipeline(int pipeline) {
    gloworm.SetPipelineIndex(pipeline);
}

void VisionSubsystem::SetDriverMode(bool driverMode) {
    gloworm.SetDriverMode(driverMode);
}

void VisionSubsystem::SetLedMode(photonlib::LEDMode mode) {
    gloworm.SetLEDMode(mode);
}

units::second_t VisionSubsystem::VisionSubsystem::GetLatency() {
    return latency;
}

units::degree_t VisionSubsystem::GetYawToTarget() {
    auto yaw = -bestTarget.GetYaw();
    frc::SmartDashboard::PutNumber("Best Target Yaw", yaw);
    return units::degree_t(yaw);
}

units::degree_t VisionSubsystem::GetPitchToTarget() {
    auto pitch = bestTarget.GetPitch();
    frc::SmartDashboard::PutNumber("Best Target Pitch", pitch);
    return units::degree_t(pitch);
}

units::degree_t VisionSubsystem::GetSkewOfTarget() {
    return units::degree_t(bestTarget.GetSkew());
}

units::meter_t VisionSubsystem::GetDistanceToTarget() {
    auto dist = photonlib::PhotonUtils::CalculateDistanceToTarget(
        str::vision_vars::CAMERA_HEIGHT, str::vision_vars::TARGET_HEIGHT,
        str::vision_vars::CAMERA_PITCH, GetPitchToTarget());
    frc::SmartDashboard::PutNumber("Distance to Target", dist.to<double>());
    return dist;
}

frc::Translation2d VisionSubsystem::GetTranslationToTarget() {
    return photonlib::PhotonUtils::EstimateCameraToTargetTranslation(
        GetDistanceToTarget(), GetYawToTarget());
}

frc::Pose2d VisionSubsystem::GetRobotPose() {
    auto pose = photonlib::PhotonUtils::EstimateFieldToRobot(
        str::vision_vars::CAMERA_HEIGHT, str::vision_vars::TARGET_HEIGHT,
        str::vision_vars::CAMERA_PITCH, GetPitchToTarget(), frc::Rotation2d(GetYawToTarget()),
        frc::Rotation2d(driveSubsystem->GetHeading()), str::vision_vars::TARGET_POSE,
        str::vision_vars::CAMERA_TO_ROBOT);
    frc::SmartDashboard::PutNumber("Vision Estimated Pose X", pose.X().to<double>());
    frc::SmartDashboard::PutNumber("Vision Estimated Pose Y", pose.Y().to<double>());
    frc::SmartDashboard::PutNumber("Vision Estimated Pose Rot", pose.Rotation().Degrees().to<double>());
    return pose;
}