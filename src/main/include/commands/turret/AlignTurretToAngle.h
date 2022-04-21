// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TurretSubsystem.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AlignTurretToAngle : public frc2::CommandHelper<frc2::CommandBase, AlignTurretToAngle> {
public:
    AlignTurretToAngle(std::function<units::degree_t()> angle, TurretSubsystem* turretSub);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

private:
    std::function<units::degree_t()> angleToGoTo;
    TurretSubsystem* turretSubsystem;
};
