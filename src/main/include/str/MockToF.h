#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <TimeOfFlight.h>

class MockToF : public wpi::Sendable, public wpi::SendableHelper<MockToF> {
public:
    MockToF(int port);
    void IdentifySensor();
    int GetFirmwareVersion() const;
    int GetSerialNumber() const;
    bool IsRangeValid() const;
    double GetRange() const;
    double GetRangeSigma() const;
    double GetAmbientLightLevel() const;
    frc::TimeOfFlight::Status GetStatus() const;
    void SetRangingMode(frc::TimeOfFlight::RangingMode mode, int sensorPeriod);
    void SetRangeOfInterest(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY);
    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    int portNum;
    int tlX{0};
    int tlY{0};
    int brX{1};
    int brY{1};
    frc::TimeOfFlight::RangingMode currentMode{frc::TimeOfFlight::RangingMode::kShort};
    double currentDistance{12.5};
};