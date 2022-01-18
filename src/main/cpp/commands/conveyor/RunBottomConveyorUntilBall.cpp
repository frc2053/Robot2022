// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunBottomConveyorUntilBall.h"

RunBottomConveyorUntilBall::RunBottomConveyorUntilBall(ConveyorSubsystem* conveyorSub)
    : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
}

// Called when the command is initially scheduled.
void RunBottomConveyorUntilBall::Initialize() {
    conveyorSubsystem->SetBottomConveyorSpeed(1);
}

// Called repeatedly when this Command is scheduled to run
void RunBottomConveyorUntilBall::Execute() {}

// Called once the command ends or is interrupted.
void RunBottomConveyorUntilBall::End(bool interrupted) {
    conveyorSubsystem->SetBottomConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunBottomConveyorUntilBall::IsFinished() {
    return conveyorSubsystem->DoesBottomSensorSeeBall();
}
