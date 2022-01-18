// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunTopConveyorUntilBall.h"

RunTopConveyorUntilBall::RunTopConveyorUntilBall(ConveyorSubsystem* conveyorSub) : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
}

// Called when the command is initially scheduled.
void RunTopConveyorUntilBall::Initialize() {
    conveyorSubsystem->SetTopConveyorSpeed(1);
}

// Called repeatedly when this Command is scheduled to run
void RunTopConveyorUntilBall::Execute() {}

// Called once the command ends or is interrupted.
void RunTopConveyorUntilBall::End(bool interrupted) {
    conveyorSubsystem->SetTopConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunTopConveyorUntilBall::IsFinished() {
    return conveyorSubsystem->DoesTopSensorSeeBall();
}
