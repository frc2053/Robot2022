// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeUp.h"
#include <iostream>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeUp::IntakeUp(IntakeSubsystem* intakeSub) : intakeSubsystem(intakeSub) {
    AddRequirements(intakeSubsystem);
    SetName("IntakeUp");
}

// Called when the command is initially scheduled.
void IntakeUp::Initialize() {
    std::cout << "Inited IntakeUp\n";
    intakeSubsystem->PutIntakeUp();
}
