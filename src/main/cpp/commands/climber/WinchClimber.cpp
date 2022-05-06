// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/climber/WinchClimber.h"

WinchClimber::WinchClimber(std::function<double()> speed, std::function<bool()> autoLock, ClimberSubsystem* climberSub)
    : winchSpeed(speed), autoLockPressed(autoLock), climberSubsystem(climberSub) {
    AddRequirements(climberSubsystem);
}

// Called when the command is initially scheduled.
void WinchClimber::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void WinchClimber::Execute() {
    double speed = winchSpeed();

    if (autoLockPressed()) {
        if (climberSubsystem->climberMotor.GetSelectedSensorVelocity() < 0) {
            climberSubsystem->UnlockClimber();
        }
    }

    climberSubsystem->SetClimberSpeed(speed);
}

// Called once the command ends or is interrupted.
void WinchClimber::End(bool interrupted) {}

// Returns true when the command should end.
bool WinchClimber::IsFinished() {
    return false;
}
