#include "Constants.h"

namespace str {
const frc::DifferentialDriveKinematics drive_pid::DRIVE_KINEMATICS(
    physical_dims::TRACK_WIDTH);

const frc::LinearSystem<2, 2, 2> drive_pid::DRIVE_TRAIN_PLANT =
    frc::LinearSystemId::IdentifyDrivetrainSystem(drive_pid::KV, drive_pid::KA,
                                                  drive_pid::KV_ANGULAR,
                                                  drive_pid::KA_ANGULAR);
}  // namespace str
