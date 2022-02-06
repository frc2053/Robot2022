#include "str/MockToF.h"
#include <iostream>
#include <wpi/sendable/SendableBuilder.h>

MockToF::MockToF(int port) : portNum(port) {}

void MockToF::IdentifySensor() {
    std::cout << "I AM TOF SENSOR " << portNum << "\n";
}

int MockToF::GetFirmwareVersion() const {
    return 420;
}

int MockToF::GetSerialNumber() const {
    return 420;
}

bool MockToF::IsRangeValid() const {
    return true;
}

double MockToF::GetRange() const {
    return currentDistance;
}

double MockToF::GetRangeSigma() const {
    return 0.1;
}

double MockToF::GetAmbientLightLevel() const {
    return 0.25;
}

frc::TimeOfFlight::Status MockToF::GetStatus() const {
    return frc::TimeOfFlight::Status::kValid;
}

void MockToF::SetRangingMode(frc::TimeOfFlight::RangingMode mode, int sensorPeriod) {
    currentMode = mode;
}

void MockToF::SetRangeOfInterest(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    tlX = topLeftX;
    tlY = topLeftY;
    brX = bottomRightX;
    brY = bottomRightY;
}

void MockToF::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("ToFSensor");
    builder.AddDoubleProperty(
        "Distance", [this] { return GetRange(); }, [this](double distance) { currentDistance = distance; });
}