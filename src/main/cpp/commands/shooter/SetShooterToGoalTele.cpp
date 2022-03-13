// the WPILib BSD license file in the root directory of this project.

#include "commands/shooter/SetShooterToGoalTele.h"
#include "commands/shooter/SetSpeedAndWait.h"
#include "commands/shooter/SetHoodToAngleAndWait.h"
#include "commands/turret/AlignTurretToGoal.h"
#include <frc2/command/PerpetualCommand.h>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
SetShooterToGoalTele::SetShooterToGoalTele(ShooterSubsystem* shooterSub, VisionSubsystem* visionSub,
                                           HoodSubsystem* hoodSub, TurretSubsystem* turretSub)
    : shooterSubsystem(shooterSub), visionSubsystem(visionSub), hoodSubsystem(hoodSub), turretSubsystem(turretSub) {
    // clang-format off
    AddCommands(
      frc2::PerpetualCommand{SetHoodToAngleAndWait{[hoodSub, visionSub](){ return hoodSub->lookupTable->Get(visionSub->GetDistanceToTarget()).angle; }, hoodSub}},
      frc2::PerpetualCommand{SetSpeedAndWait{[hoodSub, visionSub](){ return hoodSub->lookupTable->Get(visionSub->GetDistanceToTarget()).rpm; }, shooterSubsystem}},
      frc2::PerpetualCommand{AlignTurretToGoal{turretSub, visionSub}}
    );
    // clang-format on
}
