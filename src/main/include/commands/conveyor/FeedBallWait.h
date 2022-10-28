// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ConveyorSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class FeedBallWait : public frc2::CommandHelper<frc2::CommandBase, FeedBallWait> {
public:
    FeedBallWait(std::function<bool()> fire, ConveyorSubsystem* conveyorSub);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::function<bool()> shouldFire;
    ConveyorSubsystem* conveyorSubsystem;
};