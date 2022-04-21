// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/conveyor/PukeBalls.h"

PukeBalls::PukeBalls(ConveyorSubsystem* conveyorSub, IntakeSubsystem* intakeSub)
    : conveyorSubsystem(conveyorSub), intakeSubsystem(intakeSub) {
    AddRequirements(conveyorSubsystem);
}

// Called when the command is initially scheduled.
void PukeBalls::Initialize() {
    conveyorSubsystem->SetConveyorSpeed(-1);
    conveyorSubsystem->SetFunnelSpeed(-1);
    intakeSubsystem->PutIntakeDownReverse();
}

// Called repeatedly when this Command is scheduled to run
void PukeBalls::Execute() {}

// Called once the command ends or is interrupted.
void PukeBalls::End(bool interrupted) {
    conveyorSubsystem->SetConveyorSpeed(0);
    conveyorSubsystem->SetFunnelSpeed(0);
    intakeSubsystem->PutIntakeUp();
}

// Returns true when the command should end.
bool PukeBalls::IsFinished() {
    return false;
}
