// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeABall.h"
#include "commands/shooter/BlinkVisionLedsForTime.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeABall::IntakeABall(IntakeSubsystem* intakeSub, ConveyorSubsystem* conveyorSub, VisionSubsystem* visionSub)
    : intakeSubsystem(intakeSub), conveyorSubsystem(conveyorSub), visionSubsystem(visionSub) {
    frc2::ConditionalCommand whatConveyorToRun{RunFunnelUntilBall{conveyorSubsystem},
                                               RunFunnelAndConveyorUntilTopBall{conveyorSubsystem},
                                               [conveyorSub]() { return conveyorSub->DoesTopSensorSeeBall(); }};

    frc2::SequentialCommandGroup intakeUpWhenBallDetected{
        frc2::WaitUntilCommand{[conveyorSub]() { return conveyorSub->DoesBottomSensorSeeBall(); }},
        IntakeUp{intakeSubsystem}, BlinkVisionLedsForTime{1_s, visionSubsystem}};

    frc2::ParallelCommandGroup intakeUpAndConveyor{std::move(intakeUpWhenBallDetected), std::move(whatConveyorToRun)};

    AddCommands(IntakeDown{intakeSubsystem}, std::move(intakeUpAndConveyor));
    SetName("IntakeABall");
}
