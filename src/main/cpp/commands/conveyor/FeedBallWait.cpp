// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/FeedBallWait.h"

FeedBallWait::FeedBallWait(std::function<bool()> fire, ConveyorSubsystem* conveyorSub)
    : shouldFire(fire), conveyorSubsystem(conveyorSub) {}

// Called when the command is initially scheduled.
void FeedBallWait::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FeedBallWait::Execute() {
    if (shouldFire()) {
        conveyorSubsystem->SetConveyorSpeed(1);
        conveyorSubsystem->SetFunnelSpeed(1);
    } else {
        conveyorSubsystem->SetConveyorSpeed(0);
        conveyorSubsystem->SetFunnelSpeed(0);
    }
}

// Called once the command ends or is interrupted.
void FeedBallWait::End(bool interrupted) {
    conveyorSubsystem->SetConveyorSpeed(0);
    conveyorSubsystem->SetFunnelSpeed(0);
}

// Returns true when the command should end.
bool FeedBallWait::IsFinished() {
    return false;
}
