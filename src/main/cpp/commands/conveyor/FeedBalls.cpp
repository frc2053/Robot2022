// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/FeedBalls.h"

FeedBalls::FeedBalls(ConveyorSubsystem* conveyorSub, units::second_t timeout)
    : conveyorSubsystem(conveyorSub), timeToRun(timeout) {
    if (timeout != -1_s) {
        shouldTimeout = true;
    }
    AddRequirements(conveyorSubsystem);
}

// Called when the command is initially scheduled.
void FeedBalls::Initialize() {
    if (shouldTimeout) {
        timer.Reset();
        timer.Start();
    }
    conveyorSubsystem->SetConveyorSpeed(1);
    conveyorSubsystem->SetFunnelSpeed(1);
}

// Called repeatedly when this Command is scheduled to run
void FeedBalls::Execute() {}

// Called once the command ends or is interrupted.
void FeedBalls::End(bool interrupted) {
    conveyorSubsystem->SetConveyorSpeed(0);
    conveyorSubsystem->SetFunnelSpeed(0);
    timer.Stop();
}

// Returns true when the command should end.
bool FeedBalls::IsFinished() {
    if (shouldTimeout) {
        return timer.HasElapsed(timeToRun);
    }
    return false;
}
