// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeBallTele.h"
#include "commands/conveyor/RunFunnelAndConveyorTele.h"
#include "commands/intake/IntakeDown.h"
#include <frc2/command/ScheduleCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeBallTele::IntakeBallTele(IntakeSubsystem* intakeSub, ConveyorSubsystem* conveyorSub, VisionSubsystem* visionSub) {
    RunFunnelAndConveyorTele* cmd = new RunFunnelAndConveyorTele(conveyorSub);

    frc2::ScheduleCommand funnelAndConveyor{cmd};

    AddCommands(IntakeDown{intakeSub}, funnelAndConveyor);
    SetName("IntakeBallTele");
}
