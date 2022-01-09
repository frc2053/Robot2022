#pragma once

#include <AHRS.h>
#include <frc/geometry/Rotation2d.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>
#include <units/angular_velocity.h>

namespace str {
class IMU {
   public:
    IMU();
    void Calibrate();
    void ZeroYaw();

    frc::Rotation2d GetYaw();
    units::degrees_per_second_t GetYawRate();
    units::degree_t GetOffset();

    void SetYaw(double newYaw);
    void SetRate(double newRate);
    void SetOffset(units::degree_t offset);

   private:
    AHRS navxGyro{frc::SPI::Port::kMXP};
    HAL_SimDeviceHandle simGyro;
    hal::SimDouble simGyroYaw;
    hal::SimDouble simGyroRate;
    double internalOffset;
};
}  // namespace str
