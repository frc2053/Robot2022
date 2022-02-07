// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/BlinkVisionLedsForTime.h"

BlinkVisionLedsForTime::BlinkVisionLedsForTime(units::second_t time, VisionSubsystem* visionSub)
    : visionSubsystem(visionSub) {
    AddRequirements(visionSubsystem);
    SetName("BlinkVisionLedsForTime");
}

// Called when the command is initially scheduled.
void BlinkVisionLedsForTime::Initialize() {
    timer.Reset();
    timer.Start();
    visionSubsystem->SetLedMode(photonlib::LEDMode::kBlink);
}

// Called repeatedly when this Command is scheduled to run
void BlinkVisionLedsForTime::Execute() {}

// Called once the command ends or is interrupted.
void BlinkVisionLedsForTime::End(bool interrupted) {
    timer.Stop();
    visionSubsystem->SetLedMode(photonlib::LEDMode::kOn);
}

// Returns true when the command should end.
bool BlinkVisionLedsForTime::IsFinished() {
    return timer.HasElapsed(timeToBlink);
}
