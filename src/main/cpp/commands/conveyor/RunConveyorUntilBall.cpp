// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunConveyorUntilBall.h"
#include <iostream>

RunConveyorUntilBall::RunConveyorUntilBall(ConveyorSubsystem* conveyorSub) : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
    SetName("RunConveyorUntilBall");
}

// Called when the command is initially scheduled.
void RunConveyorUntilBall::Initialize() {
    std::cout << "initialized run conveyor until ball!\n";
    conveyorSubsystem->SetConveyorSpeed(.5);
}

// Called repeatedly when this Command is scheduled to run
void RunConveyorUntilBall::Execute() {}

// Called once the command ends or is interrupted.
void RunConveyorUntilBall::End(bool interrupted) {
    std::cout << "ended run conveyor until ball!\n";
    conveyorSubsystem->SetConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunConveyorUntilBall::IsFinished() {
    return conveyorSubsystem->DoesTopSensorSeeBall();
}
