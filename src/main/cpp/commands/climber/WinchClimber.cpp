// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/climber/WinchClimber.h"

WinchClimber::WinchClimber(std::function<double()> speed, ClimberSubsystem* climberSub)
    : winchSpeed(speed), climberSubsystem(climberSub) {
    AddRequirements(climberSubsystem);
}

// Called when the command is initially scheduled.
void WinchClimber::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void WinchClimber::Execute() {
    double speed = winchSpeed();
    if (std::abs(speed) < .1) {
        speed = 0;
    }
    climberSubsystem->SetClimberSpeed(speed);
}

// Called once the command ends or is interrupted.
void WinchClimber::End(bool interrupted) {}

// Returns true when the command should end.
bool WinchClimber::IsFinished() {
    return false;
}
