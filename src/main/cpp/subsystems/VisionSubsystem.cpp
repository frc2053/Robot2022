// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"
#include <photonlib/PhotonUtils.h>
#include <iostream>

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
    gloworm_sim.ProcessFrame(driveSubsystem->GetPose());
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
    return units::degree_t(-bestTarget.GetYaw());
}

units::degree_t VisionSubsystem::GetPitchToTarget() {
    return units::degree_t(bestTarget.GetPitch());
}

units::degree_t VisionSubsystem::GetSkewOfTarget() {
    return units::degree_t(bestTarget.GetSkew());
}

units::meter_t VisionSubsystem::GetDistanceToTarget() {
    return photonlib::PhotonUtils::CalculateDistanceToTarget(
        str::vision_vars::CAMERA_HEIGHT, str::vision_vars::TARGET_HEIGHT,
        str::vision_vars::CAMERA_PITCH, GetPitchToTarget());
}

frc::Translation2d VisionSubsystem::GetTranslationToTarget() {
    return photonlib::PhotonUtils::EstimateCameraToTargetTranslation(
        GetDistanceToTarget(), GetYawToTarget());
}

frc::Pose2d VisionSubsystem::GetRobotPose() {
    return photonlib::PhotonUtils::EstimateFieldToRobot(
        str::vision_vars::CAMERA_HEIGHT, str::vision_vars::TARGET_HEIGHT,
        str::vision_vars::CAMERA_PITCH, GetPitchToTarget(), frc::Rotation2d(GetYawToTarget()),
        frc::Rotation2d(driveSubsystem->GetHeading()), str::vision_vars::TARGET_POSE,
        str::vision_vars::CAMERA_TO_ROBOT);
}