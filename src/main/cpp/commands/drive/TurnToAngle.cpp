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
TurnToAngle::TurnToAngle(units::degree_t target, DrivetrainSubsystem* drive)
    : CommandHelper(
          frc::PIDController(str::drive_pid::TURN_P, str::drive_pid::TURN_I,
                             str::drive_pid::TURN_D),
          [drive] { return drive->GetHeading().to<double>(); },
          // This should return the goal state (can also be a constant)
          target.to<double>(),
          // This uses the output and current trajectory setpoint
          [drive](double output) { drive->ArcadeDrive(0, output); }, {drive}) {
    GetController().EnableContinuousInput(-180, 180);
    GetController().SetTolerance(
        str::drive_pid::TURN_TOLERANCE.to<double>(),
        str::drive_pid::TURN_RATE_TOLERANCE.to<double>());
    AddRequirements({drive});
}

// Returns true when the command should end.
bool TurnToAngle::IsFinished() {
    return GetController().AtSetpoint();
}
