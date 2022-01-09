// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/auto/AroundTheField.h"
#include <frc2/command/InstantCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AroundTheField::AroundTheField(DrivetrainSubsystem* drivetrainSub)
    : m_drivetrainSub(drivetrainSub) {
    AddCommands(frc2::InstantCommand(
                    [this]() { m_drivetrainSub->SetGyroOffset(90_deg); }),
                std::move(turn1), std::move(turn2));
}
