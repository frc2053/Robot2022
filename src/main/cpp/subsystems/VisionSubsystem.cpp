// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>

VisionSubsystem::VisionSubsystem(DrivetrainSubsystem* driveSub, TurretSubsystem* turretSub)
    : driveSubsystem(driveSub), turretSubsystem(turretSub) {
    SetName("VisionSubsystem");
    gloworm_sim.AddSimVisionTarget(
        photonlib::SimVisionTarget(str::vision_vars::TARGET_POSE_ONE, str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND,
                                   str::vision_vars::TARGET_WIDTH, str::vision_vars::TARGET_HEIGHT));
    gloworm_sim.AddSimVisionTarget(
        photonlib::SimVisionTarget(str::vision_vars::TARGET_POSE_TWO, str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND,
                                   str::vision_vars::TARGET_WIDTH, str::vision_vars::TARGET_HEIGHT));
    gloworm_sim.AddSimVisionTarget(
        photonlib::SimVisionTarget(str::vision_vars::TARGET_POSE_THREE, str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND,
                                   str::vision_vars::TARGET_WIDTH, str::vision_vars::TARGET_HEIGHT));
    gloworm_sim.AddSimVisionTarget(
        photonlib::SimVisionTarget(str::vision_vars::TARGET_POSE_FOUR, str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND,
                                   str::vision_vars::TARGET_WIDTH, str::vision_vars::TARGET_HEIGHT));
}

// This method will be called once per scheduler run
void VisionSubsystem::Periodic() {
    auto cam_to_bot = turretSubsystem->GetCameraToRobotPose();
    driveSubsystem->DrawTurret(cam_to_bot);
    gloworm_sim.MoveCamera(cam_to_bot, str::vision_vars::CAMERA_HEIGHT, str::vision_vars::CAMERA_PITCH);
    frc::SmartDashboard::PutNumber("cam_to_bot_rot", cam_to_bot.Rotation().Degrees().value());
    latestData = gloworm.GetLatestResult();
    if (latestData.HasTargets()) {
        seesATarget = true;
        targetsFound = latestData.GetTargets();
        bestTarget = latestData.GetBestTarget();
        latency = latestData.GetLatency();
        // driveSubsystem->AddVisionMeasurement(GetRobotPose(), latency);
    } else {
        seesATarget = false;
    }
}

void VisionSubsystem::SimulationPeriodic() {
    auto dt_pose = driveSubsystem->GetPose();
    gloworm_sim.ProcessFrame(dt_pose);
}

void VisionSubsystem::ChangeVisionPipeline(int pipeline) {
    gloworm.SetPipelineIndex(pipeline);
}
// robot.win();
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
    auto dist = photonlib::PhotonUtils::CalculateDistanceToTarget(str::vision_vars::CAMERA_HEIGHT,
                                                                  str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND,
                                                                  str::vision_vars::CAMERA_PITCH, GetPitchToTarget());
    return dist;
}

frc::Translation2d VisionSubsystem::GetTranslationToTarget() {
    auto translationToTarget =
        photonlib::PhotonUtils::EstimateCameraToTargetTranslation(GetDistanceToTarget(), GetYawToTarget());

    return translationToTarget;
}

frc::Pose2d VisionSubsystem::GetRobotPose() {
    auto pose = photonlib::PhotonUtils::EstimateFieldToRobot(
        str::vision_vars::CAMERA_HEIGHT, str::vision_vars::TARGET_HEIGHT_ABOVE_GROUND, str::vision_vars::CAMERA_PITCH,
        GetPitchToTarget(), frc::Rotation2d(GetYawToTarget()), frc::Rotation2d(driveSubsystem->GetHeading()),
        str::vision_vars::TARGET_POSE_ONE, str::vision_vars::CAMERA_TO_ROBOT);
    return pose;
}

bool VisionSubsystem::SeesTarget() {
    return seesATarget;
}