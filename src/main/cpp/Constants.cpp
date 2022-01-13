#include "Constants.h"

namespace str {
const frc::DifferentialDriveKinematics drive_pid::DRIVE_KINEMATICS(
    physical_dims::TRACK_WIDTH);

const frc::LinearSystem<2, 2, 2> drive_pid::DRIVE_TRAIN_PLANT =
    frc::LinearSystemId::IdentifyDrivetrainSystem(drive_pid::KV, drive_pid::KA,
                                                  drive_pid::KV_ANGULAR,
                                                  drive_pid::KA_ANGULAR);

frc::LinearSystem<1, 1, 1> shooter_pid::SHOOTER_PLANT = 
    frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(shooter_pid::KV, shooter_pid::KA);
}  // namespace str
