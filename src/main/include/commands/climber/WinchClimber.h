// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include "subsystems/ClimberSubsystem.h"
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class WinchClimber : public frc2::CommandHelper<frc2::CommandBase, WinchClimber> {
public:
    WinchClimber(std::function<double()> speed, std::function<bool()> autoLock, ClimberSubsystem* climberSub);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::function<double()> winchSpeed;
    std::function<bool()> autoLockPressed;
    ClimberSubsystem* climberSubsystem;
};
