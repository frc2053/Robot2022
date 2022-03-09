// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunFunnelAndConveyorTele.h"
#include <iostream>

RunFunnelAndConveyorTele::RunFunnelAndConveyorTele(ConveyorSubsystem* conveyorSub) : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
    SetName("RunFunnelAndConveyorTele");
}

// Called when the command is initially scheduled.
void RunFunnelAndConveyorTele::Initialize() {
    std::cout << "initialized tele funnel and conveyor\n";
    conveyorSubsystem->SetFunnelSpeed(1);
    conveyorSubsystem->SetConveyorSpeed(.5);
    isDone = false;
    hasSeenBallAtTop = false;
}

// Called repeatedly when this Command is scheduled to run
void RunFunnelAndConveyorTele::Execute() {
    if (conveyorSubsystem->DoesTopSensorSeeBall()) {
        conveyorSubsystem->SetConveyorSpeed(0);
        hasSeenBallAtTop = true;
        std::cout << "Seen ball at top!\n";
    }
    if (conveyorSubsystem->DoesBottomSensorSeeBall()) {
        std::cout << "Seen ball at bottom!\n";
        if (hasSeenBallAtTop) {
            isDone = true;
            std::cout << "We are done!\n";
        }
    }
}

// Called once the command ends or is interrupted.
void RunFunnelAndConveyorTele::End(bool interrupted) {
    std::cout << "ended tele funnel and conveyor\n";
    conveyorSubsystem->SetConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunFunnelAndConveyorTele::IsFinished() {
    return isDone;
}
