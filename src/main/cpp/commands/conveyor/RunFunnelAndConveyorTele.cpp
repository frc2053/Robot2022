// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunFunnelAndConveyorTele.h"
#include <iostream>

RunFunnelAndConveyorTele::RunFunnelAndConveyorTele(ConveyorSubsystem* conveyorSub) : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
    SetName("RunFunnelAndConveyorUntilTopBall");
}

// Called when the command is initially scheduled.
void RunFunnelAndConveyorTele::Initialize() {
    std::cout << "initialized tele funnel and conveyor\n";
    conveyorSubsystem->SetFunnelSpeed(1);
    conveyorSubsystem->SetConveyorSpeed(.5);
}

// Called repeatedly when this Command is scheduled to run
void RunFunnelAndConveyorTele::Execute() {
    if (conveyorSubsystem->DoesTopSensorSeeBall()) {
        hasTopBall = true;
        conveyorSubsystem->SetConveyorSpeed(0);
    }
    if (conveyorSubsystem->DoesBottomSensorSeeBall()) {
        if (hasTopBall) {
            conveyorSubsystem->SetFunnelSpeed(0);
        }
    }
}

// Called once the command ends or is interrupted.
void RunFunnelAndConveyorTele::End(bool interrupted) {
    std::cout << "ended tele funnel and conveyor\n";
    conveyorSubsystem->SetFunnelSpeed(0);
    conveyorSubsystem->SetConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunFunnelAndConveyorTele::IsFinished() {
    return false;
}
