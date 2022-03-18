// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/HoodSubsystem.h"
#include "str/Units.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>

HoodSubsystem::HoodSubsystem(str::ShooterLookupTable* shooterTable, VisionSubsystem* visionSub)
    : lookupTable(shooterTable), visionSubsystem(visionSub) {
    hoodEncoder.SetSamplesToAverage(50);
    hoodEncoder.SetMinRate(1.0);
    hoodEncoder.SetMaxPeriod(.5_s);
    hoodEncoder.SetDistancePerPulse(0.023162583518931);
    hoodEncoder.SetReverseDirection(true);
    hoodController.SetSetpoint(str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value());
    hoodController.SetTolerance(str::shooter_pid::HOOD_TOLERANCE);
    frc::SmartDashboard::PutData("Hood PID", &hoodController);
}

// This method will be called once per scheduler run
void HoodSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Hood Angle", GetHoodAngle().value());
    double servoSetSpeed = hoodController.Calculate(GetHoodAngle().to<double>());

    if (servoSetSpeed > 0 && hoodEncoder.GetStopped()) {
        std::cout << "The hood is commanded forward but we are stopped!\n";
        if (!startedTimerF) {
            std::cout << "The forward stall timer is started!\n";
            stallTimerF.Reset();
            stallTimerF.Start();
            startedTimerF = true;
        } else {
            if (stallTimerF.HasElapsed(.5_s)) {
                std::cout << "Timer elapsed for forward, stopping motor!\n";
                servoSetSpeed = 0;
                stallTimerF.Stop();
                stallTimerF.Reset();
                startedTimerF = false;
                hoodEncoder.Reset();
                hoodOffset = 52_deg;
                hoodController.Reset();
            }
        }
    }
    if (servoSetSpeed < 0 && hoodEncoder.GetStopped()) {
        std::cout << "The hood is commanded backwards but we are stopped!\n";
        if (!startedTimerB) {
            std::cout << "The backwards stall timer is started!\n";
            stallTimerB.Reset();
            stallTimerB.Start();
            startedTimerB = true;
        } else {
            if (stallTimerB.HasElapsed(.5_s)) {
                std::cout << "Timer elapsed for backwards, stopping motor!\n";
                servoSetSpeed = 0;
                stallTimerB.Stop();
                stallTimerB.Reset();
                startedTimerB = false;
                hoodEncoder.Reset();
                hoodOffset = 0_deg;
                hoodController.Reset();
            }
        }
    }

    SetServoSpeed(std::clamp(servoSetSpeed, -1.0, 1.0));

    // double outputSpeed = hoodOutputVal;

    // if (hoodOutputVal < 0) {
    //     stallForwards = false;
    //     if (hoodEncoder.GetRate() < -5) {
    //         stallBackwards = true;
    //     } else {
    //         stallBackwards = false;
    //     }
    // } else if (hoodOutputVal > 0) {
    //     stallBackwards = false;
    //     if (hoodEncoder.GetRate() < 5) {
    //         stallForwards = true;
    //     } else {
    //         stallForwards = false;
    //     }
    // }

    // if (stallForwards) {
    //     if (startedTimerF) {
    //         if (stallTimerF.HasElapsed(.5_s)) {
    //             outputSpeed = 0;
    //             hoodEncoder.Reset();
    //             hoodController.SetSetpoint(0);
    //         }
    //     } else {
    //         startedTimerF = true;
    //         stallTimerF.Start();
    //     }
    // } else {
    //     stallTimerF.Stop();
    //     startedTimerF = false;
    //     stallTimerF.Reset();
    // }

    // if (stallBackwards) {
    //     if (startedTimerB) {
    //         if (stallTimerB.HasElapsed(.5_s)) {
    //             outputSpeed = 0;
    //         }
    //     } else {
    //         startedTimerB = true;
    //         stallTimerB.Start();
    //     }
    // } else {
    //     stallTimerB.Stop();
    //     startedTimerB = false;
    //     stallTimerB.Reset();
    // }
}

int HoodSubsystem::ConvertHoodAngleToTicks(units::degree_t angle) {
    return str::Units::map(angle.to<double>(), 0, str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value(), 0,
                           str::shooter_pid::SHOOTER_HOOD_MAX_TICKS);
}

units::degree_t HoodSubsystem::ConvertHoodTicksToAngle(int ticks) {
    return units::degree_t(str::Units::map(ticks, 0, str::shooter_pid::SHOOTER_HOOD_MAX_TICKS, 0,
                                           str::shooter_pid::SHOOTER_HOOD_MAX_ANGLE.value()));
}

units::degree_t HoodSubsystem::GetHoodAngle() {
    return units::degree_t(hoodEncoder.Get()) + hoodOffset;
}

void HoodSubsystem::SetHoodToAngle(units::degree_t setpoint) {
    hoodController.SetSetpoint(setpoint.to<double>());
}

void HoodSubsystem::SetServoSpeed(double percent) {
    hoodServo.Set(str::Units::map(percent, -1, 1, 0, 1));
}

units::degree_t HoodSubsystem::GetHoodAngleToGoTo() {
    return hoodAngleToGoTo;
}

bool HoodSubsystem::IsHoodAtSetpoint() {
    return hoodController.AtSetpoint();
}