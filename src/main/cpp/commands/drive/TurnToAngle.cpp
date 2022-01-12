// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/drive/TurnToAngle.h"

#include <units/acceleration.h>
#include <units/velocity.h>
#include "Constants.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TurnToAngle::TurnToAngle(std::function<units::radian_t()> target, DrivetrainSubsystem* drive)
    : CommandHelper(
          frc::ProfiledPIDController<units::radians>(
          str::drive_pid::TURN_P, str::drive_pid::TURN_I, str::drive_pid::TURN_D, {str::drive_pid::MAX_TURN_RATE, str::drive_pid::MAX_TURN_ACCEL}),
          [drive] { return drive->GetHeading(); },
          // This should return the goal state (can also be a constant)
          target,
          // This uses the output and current trajectory setpoint
          [drive](double output, auto setPointState) { drive->ArcadeDrive(0, output); }, 
          {drive}) {
    GetController().EnableContinuousInput(-180_deg, 180_deg);
    GetController().SetTolerance(
        str::drive_pid::TURN_TOLERANCE,
        str::drive_pid::TURN_RATE_TOLERANCE);
    AddRequirements({drive});
    SetName("TurnToAngle");
}

// Returns true when the command should end.
bool TurnToAngle::IsFinished() {
    return GetController().AtGoal();
}
