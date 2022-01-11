// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DrivetrainSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopDrive : public frc2::CommandHelper<frc2::CommandBase, TeleopDrive> {
   public:
    TeleopDrive(std::function<double()> fow, std::function<double()> rot, std::function<bool()> quickTurn, DrivetrainSubsystem* driveSub);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;
private:
    DrivetrainSubsystem* drivetrainSub;
    std::function<double()> forwardCmd;
    std::function<double()> rotCmd;
    std::function<bool()> quickTurnCmd;
};
