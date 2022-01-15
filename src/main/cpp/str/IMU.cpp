#include "str/IMU.h"
#include <frc/RobotBase.h>
#include <iostream>

str::IMU::IMU() {
    if (frc::RobotBase::IsSimulation()) {
        simGyro = HALSIM_GetSimDeviceHandle("navX-Sensor[4]");
        simGyroYaw = HALSIM_GetSimValueHandle(simGyro, "Yaw");
        simGyroRate = HALSIM_GetSimValueHandle(simGyro, "Rate");
        std::cout << "Gyro is running in sim!\n";
    } else {
        std::cout << "Initialized real robot gyro!\n";
        simGyro = -1;
        simGyroYaw = -1;
        simGyroRate = -1;
    }
    navxGyro.Calibrate();
    navxGyro.ZeroYaw();
    std::cout << "Finished initialization of gyro.\n";
}

void str::IMU::Calibrate() {
    std::cout << "Calibrated gyro.\n";
    navxGyro.Calibrate();
}

void str::IMU::ZeroYaw() {
    std::cout << "Zeroed gyro.\n";
    navxGyro.ZeroYaw();
    if (frc::RobotBase::IsSimulation()) {
        simGyroYaw.Set(0);
    }
}

frc::Rotation2d str::IMU::GetYaw() {
    units::degree_t gyroVal =
        units::degree_t(-navxGyro.GetYaw() + internalOffset);
    frc::Rotation2d retVal = frc::Rotation2d(gyroVal);
    return retVal;
}

units::degrees_per_second_t str::IMU::GetYawRate() {
    return units::degrees_per_second_t(-navxGyro.GetRate());
}

units::degree_t str::IMU::GetOffset() {
    return units::degree_t(internalOffset);
}

void str::IMU::SetYaw(double newYaw) {
    if (frc::RobotBase::IsSimulation()) {
        simGyroYaw.Set(-newYaw + internalOffset);
    } else {
        std::cout << "You tried setting the gyro yaw not in simulation. This "
                     "call did not do anything.\n";
    }
}

void str::IMU::SetRate(double newRate) {
    if (frc::RobotBase::IsSimulation()) {
        simGyroYaw.Set(-newRate);
    } else {
        std::cout << "You tried setting the gyro rate not in simulation. This "
                     "call did not do anything.\n";
    }
}

void str::IMU::SetOffset(units::degree_t offset) {
    internalOffset = offset.to<double>();
}