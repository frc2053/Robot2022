// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include "commands/intake/IntakeDown.h"
#include "commands/intake/IntakeUp.h"
#include <frc2/command/ConditionalCommand.h>
#include "commands/conveyor/RunFunnelUntilBall.h"
#include "commands/conveyor/RunFunnelAndConveyorUntilTopBall.h"
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/WaitUntilCommand.h>
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ConveyorSubsystem.h"

class IntakeABall : public frc2::CommandHelper<frc2::SequentialCommandGroup, IntakeABall> {
public:
    IntakeABall(IntakeSubsystem* intakeSubsystem, ConveyorSubsystem* conveyorSubsystem);

private:
    IntakeSubsystem* intakeSubsystem;
    ConveyorSubsystem* conveyorSubsystem;

    frc2::ConditionalCommand whatConveyorToRun{std::move(RunFunnelUntilBall(conveyorSubsystem)),
                                               std::move(RunFunnelAndConveyorUntilTopBall(conveyorSubsystem)),
                                               [this]() { return conveyorSubsystem->DoesTopSensorSeeBall(); }};

    frc2::SequentialCommandGroup intakeUpWhenBallDetected{
        frc2::WaitUntilCommand([this]() { return conveyorSubsystem->DoesBottomSensorSeeBall(); }),
        IntakeUp(intakeSubsystem)};

    frc2::ParallelCommandGroup intakeUpAndConveyor{std::move(intakeUpWhenBallDetected), std::move(whatConveyorToRun)};
};
