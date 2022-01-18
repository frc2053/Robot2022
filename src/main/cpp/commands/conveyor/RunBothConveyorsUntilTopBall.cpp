// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunBothConveyorsUntilTopBall.h"

RunBothConveyorsUntilTopBall::RunBothConveyorsUntilTopBall(ConveyorSubsystem* conveyorSub)
    : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
}

// Called when the command is initially scheduled.
void RunBothConveyorsUntilTopBall::Initialize() {
    conveyorSubsystem->SetTopConveyorSpeed(1);
    conveyorSubsystem->SetBottomConveyorSpeed(1);
}

// Called repeatedly when this Command is scheduled to run
void RunBothConveyorsUntilTopBall::Execute() {}

// Called once the command ends or is interrupted.
void RunBothConveyorsUntilTopBall::End(bool interrupted) {
    conveyorSubsystem->SetTopConveyorSpeed(0);
    conveyorSubsystem->SetBottomConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunBothConveyorsUntilTopBall::IsFinished() {
    return conveyorSubsystem->DoesTopSensorSeeBall();
}
