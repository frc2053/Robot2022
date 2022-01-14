// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/plant/LinearSystemId.h>

#include <units/acceleration.h>
#include <units/length.h>
#include <units/voltage.h>

namespace str {

namespace auto_consts {
static constexpr auto K_MAX_SPEED = 15.9_fps;
static constexpr auto K_MAX_ACCEL = 20_fps_sq;
static constexpr auto K_RAMSETE_B = 2 * 1_rad * 1_rad / (1_m * 1_m);
static constexpr auto K_RAMSETE_ZETA = 0.7 / 1_rad;
}  // namespace auto_consts

namespace can_ids {
static constexpr int FRONT_LEFT_DRIVEBASE_TALON_ID = 2;
static constexpr int REAR_LEFT_DRIVEBASE_TALON_ID = 3;
static constexpr int FRONT_RIGHT_DRIVEBASE_TALON_ID = 4;
static constexpr int REAR_RIGHT_DRIVEBASE_TALON_ID = 5;
static constexpr int SHOOTERLEADER_TALON_ID = 6;
static constexpr int SHOOTERFOLLOWER01_TALON_ID = 7;
static constexpr int SHOOTERFOLLOWER02_TALON_ID = 8;
}  // namespace can_ids

namespace drive_pid {
extern const frc::DifferentialDriveKinematics DRIVE_KINEMATICS;
extern const frc::LinearSystem<2, 2, 2> DRIVE_TRAIN_PLANT;

static constexpr double KF = 0.0;
static constexpr double KP = 8.5;
static constexpr double KI = 0;
static constexpr double KD = 0;

static constexpr auto KS = 0.22_V;
static constexpr auto KV = 1.98 * 1_V / 1_mps;
static constexpr auto KA = 0.2 * 1_V / 1_mps_sq;
static constexpr auto KV_ANGULAR = 1.5 * 1_V / 1_mps;
static constexpr auto KA_ANGULAR = 0.3 * 1_V / 1_mps_sq;

static constexpr double TURN_P = 0.1;
static constexpr double TURN_I = 0;
static constexpr double TURN_D = 0;
static constexpr auto TURN_TOLERANCE = 2_deg;
static constexpr auto TURN_RATE_TOLERANCE = 10_deg_per_s;
constexpr auto MAX_TURN_RATE = 100_deg_per_s;
constexpr auto MAX_TURN_ACCEL = 300_deg_per_s / 1_s;
}  // namespace drive_pid

namespace shooter_pid {
extern frc::LinearSystem<1, 1, 1> SHOOTER_PLANT;
static constexpr auto KV = 0.02_V / 1_rad_per_s;
static constexpr auto KA = 0.01_V / 1_rad_per_s_sq;
}  // namespace shooter_pid

namespace encoder_cpr {
static constexpr int TALON_FX_ENCODER_CPR = 2048;
static constexpr int CANCODER_ENCODER_CPR = 4096;
static constexpr int NEO_ENCODER_CPR = 48;
}  // namespace encoder_cpr

namespace oi {
static constexpr int DRIVER_CONTROLLER_PORT = 0;
static constexpr int OPERATOR_CONTROLLER_PORT = 1;
}  // namespace oi

namespace physical_dims {
static constexpr auto TRACK_WIDTH = 27_in;
static constexpr auto DRIVE_WHEEL_DIAMETER = 4_in;
static constexpr double DRIVEBASE_GEARBOX_RATIO = 7.0;
static constexpr auto DRIVEBASE_GEARBOX = frc::DCMotor::Falcon500(2);

static constexpr auto SHOOTER_GEARBOX = frc::DCMotor::Falcon500(2);
static constexpr double SHOOTER_GEARBOX_RATIO = 0.5;
static constexpr auto SHOOTER_WHEEL_DIAMETER = 4_in;
}  // namespace physical_dims

namespace vision_vars {
static constexpr units::meter_t CAMERA_HEIGHT = 3_ft;
static constexpr units::meter_t TARGET_HEIGHT_ABOVE_GROUND = 103.619_in;
static constexpr units::meter_t TARGET_HEIGHT = 2_in;
static constexpr units::meter_t TARGET_WIDTH = 53_in;
static constexpr units::degree_t CAMERA_PITCH = 20_deg;
static frc::Pose2d TARGET_POSE = frc::Pose2d(27_ft, 13.5_ft, 0_deg);
static const frc::Transform2d CAMERA_TO_ROBOT = frc::Transform2d(frc::Translation2d(0_ft, 0_ft), frc::Rotation2d(0_deg));
static constexpr units::degree_t CAMERA_DIAG_FOV = 74.8_deg;
static constexpr units::meter_t MAX_LED_RANGE = 20_m;
static constexpr int GLOWORM_RES_X = 640;
static constexpr int GLOWORM_REX_Y = 480;
static constexpr double MIN_TARGET_AREA = 10; //square pixels
}  // namespace vision_vars
}  // namespace str
