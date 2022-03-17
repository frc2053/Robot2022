// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/HoodSubsystem.h"
#include "str/Units.h"
#include "frc/smartdashboard/SmartDashboard.h"

HoodSubsystem::HoodSubsystem(str::ShooterLookupTable* shooterTable, VisionSubsystem* visionSub)
    : lookupTable(shooterTable), visionSubsystem(visionSub) {
    hoodEncoder.SetSamplesToAverage(50);
    hoodEncoder.SetMinRate(1.0);
    hoodEncoder.SetDistancePerPulse(1);
    hoodEncoder.SetReverseDirection(true);
    hoodController.SetSetpoint(str::shooter_pid::SHOOTER_HOOD_MAX_TICKS);
    hoodController.SetTolerance(ConvertHoodAngleToTicks(units::degree_t(str::shooter_pid::HOOD_TOLERANCE)));
    frc::SmartDashboard::PutData("Hood PID", &hoodController);
}

// This method will be called once per scheduler run
void HoodSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Hood Angle", GetHoodAngle().value());

    str::LookupValue goalTarget = lookupTable->Get(visionSubsystem->GetDistanceToTarget());
    hoodAngleToGoTo = goalTarget.angle;

    double hoodOutputVal = hoodController.Calculate(GetHoodAngle().to<double>());

    double outputSpeed = hoodOutputVal;

    if (hoodOutputVal < 0) {
        stallForwards = false;
        if (hoodEncoder.GetRate() < -5) {
            stallBackwards = true;
        } else {
            stallBackwards = false;
        }
    } else if (hoodOutputVal > 0) {
        stallBackwards = false;
        if (hoodEncoder.GetRate() < 5) {
            stallForwards = true;
        } else {
            stallForwards = false;
        }
    }

    if (stallForwards) {
        if (startedTimerF) {
            if (stallTimerF.HasElapsed(.5_s)) {
                outputSpeed = 0;
                hoodEncoder.Reset();
                hoodController.SetSetpoint(0);
            }
        } else {
            startedTimerF = true;
            stallTimerF.Start();
        }
    } else {
        stallTimerF.Stop();
        startedTimerF = false;
        stallTimerF.Reset();
    }

    if (stallBackwards) {
        if (startedTimerB) {
            if (stallTimerB.HasElapsed(.5_s)) {
                outputSpeed = 0;
            }
        } else {
            startedTimerB = true;
            stallTimerB.Start();
        }
    } else {
        stallTimerB.Stop();
        startedTimerB = false;
        stallTimerB.Reset();
    }

    SetServoSpeed(outputSpeed);
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
    return ConvertHoodTicksToAngle(hoodEncoder.Get() + tickOffset);
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