// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeUpTele.h"
#include "commands/intake/IntakeUp.h"
#include <frc2/command/WaitCommand.h>
#include <iostream>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeUpTele::IntakeUpTele(IntakeSubsystem* intakeSub, ConveyorSubsystem* conveyorSub) {
    AddCommands(IntakeUp(intakeSub), frc2::WaitCommand(.5_s),
                frc2::InstantCommand(
                    [conveyorSub]() {
                        std::cout << "Stopped funnel!\n";
                        conveyorSub->SetFunnelSpeed(0);
                    },
                    {conveyorSub}));
}
