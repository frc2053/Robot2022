// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <iostream>
#include "commands/conveyor/RunFunnelUntilBall.h"

RunFunnelUntilBall::RunFunnelUntilBall(ConveyorSubsystem* conveyorSub) : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
    SetName("RunFunnelUntilBall");
}

// Called when the command is initially scheduled.
void RunFunnelUntilBall::Initialize() {
    conveyorSubsystem->SetFunnelSpeed(1);
    std::cout << "initialized run funnel until ball!\n";
}

// Called repeatedly when this Command is scheduled to run
void RunFunnelUntilBall::Execute() {}

// Called once the command ends or is interrupted.
void RunFunnelUntilBall::End(bool interrupted) {
    conveyorSubsystem->SetFunnelSpeed(0);
    std::cout << "ended run funnel until ball!\n";
}

// Returns true when the command should end.
bool RunFunnelUntilBall::IsFinished() {
    return conveyorSubsystem->DoesBottomSensorSeeBall();
}
