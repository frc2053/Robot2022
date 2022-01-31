// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/RunFunnelAndConveyorUntilTopBall.h"

RunFunnelAndConveyorUntilTopBall::RunFunnelAndConveyorUntilTopBall(ConveyorSubsystem* conveyorSub)
    : conveyorSubsystem(conveyorSub) {
    AddRequirements(conveyorSubsystem);
}

// Called when the command is initially scheduled.
void RunFunnelAndConveyorUntilTopBall::Initialize() {
    conveyorSubsystem->SetFunnelSpeed(1);
    conveyorSubsystem->SetConveyorSpeed(1);
}

// Called repeatedly when this Command is scheduled to run
void RunFunnelAndConveyorUntilTopBall::Execute() {}

// Called once the command ends or is interrupted.
void RunFunnelAndConveyorUntilTopBall::End(bool interrupted) {
    conveyorSubsystem->SetFunnelSpeed(0);
    conveyorSubsystem->SetConveyorSpeed(0);
}

// Returns true when the command should end.
bool RunFunnelAndConveyorUntilTopBall::IsFinished() {
    return conveyorSubsystem->DoesTopSensorSeeBall();
}