// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
public:
    ClimberSubsystem();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    void ExtendClimber();
    void RetractClimber();
    void LockClimber();
    void UnlockClimber();

private:
    frc::DoubleSolenoid climberSolenoid{frc::PneumaticsModuleType::CTREPCM, str::pcm_ports::CLIMBER_SOLENOID_PORT1,
                                        str::pcm_ports::CLIMBER_SOLENOID_PORT2};
    frc::DoubleSolenoid climberLockSolenoid{frc::PneumaticsModuleType::CTREPCM, str::pcm_ports::CLIMBER_LOCK_PORT1,
                                            str::pcm_ports::CLIMBER_LOCK_PORT2};
};
