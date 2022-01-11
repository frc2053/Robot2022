// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive/TeleopDrive.h"

TeleopDrive::TeleopDrive(std::function<double()> fow,
                         std::function<double()> rot,
                         std::function<bool()> quickTurn,
                         DrivetrainSubsystem* driveSub)
    : forwardCmd(fow), rotCmd(rot), quickTurnCmd(quickTurn), drivetrainSub(driveSub) {
    AddRequirements(drivetrainSub);
    SetName("TeleopDrive");
}

// Called when the command is initially scheduled.
void TeleopDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopDrive::Execute() {
    drivetrainSub->CurvatureDrive(forwardCmd(), rotCmd(), quickTurnCmd());
}

// Called once the command ends or is interrupted.
void TeleopDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool TeleopDrive::IsFinished() {
    return false;
}
