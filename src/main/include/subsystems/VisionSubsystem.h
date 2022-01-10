// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include "subsystems/DrivetrainSubsystem.h"
#include <photonlib/SimVisionSystem.h>
#include "Constants.h"


class VisionSubsystem : public frc2::SubsystemBase {
   public:
    VisionSubsystem(DrivetrainSubsystem* driveSub);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void SimulationPeriodic() override;
    void SavePicture();
    void ChangeVisionPipeline(int pipeline);
    void SetDriverMode(bool driverMode);
    void SetLedMode(photonlib::LEDMode mode);
    units::second_t GetLatency();
    units::degree_t GetYawToTarget();
    units::degree_t GetPitchToTarget();
    units::degree_t GetSkewOfTarget();
    units::meter_t GetDistanceToTarget();
    frc::Translation2d GetTranslationToTarget();
    frc::Pose2d GetRobotPose();
   private:
    DrivetrainSubsystem* driveSubsystem;
    photonlib::PhotonCamera gloworm{"gloworm-cam"};
    photonlib::PhotonPipelineResult latestData;
    units::second_t latency;
    wpi::span<const photonlib::PhotonTrackedTarget> targetsFound;
    photonlib::PhotonTrackedTarget bestTarget;
    photonlib::SimVisionSystem gloworm_sim{
        "gloworm-cam",
        str::vision_vars::CAMERA_DIAG_FOV,
        str::vision_vars::CAMERA_PITCH,
        str::vision_vars::CAMERA_TO_ROBOT,
        str::vision_vars::CAMERA_HEIGHT,
        str::vision_vars::MAX_LED_RANGE,
        str::vision_vars::GLOWORM_RES_X,
        str::vision_vars::GLOWORM_REX_Y,
        str::vision_vars::MIN_TARGET_AREA
    };
};
