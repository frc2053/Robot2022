// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ConveyorSubsystem.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FeedBalls : public frc2::CommandHelper<frc2::CommandBase, FeedBalls> {
public:
    FeedBalls(ConveyorSubsystem* conveyorSub, units::second_t timeout = -1_s);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    ConveyorSubsystem* conveyorSubsystem;
    frc::Timer timer;
    bool shouldTimeout{false};
    units::second_t timeToRun;
};
