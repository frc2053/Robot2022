// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/intake/IntakeDown.h"
#include <iostream>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
IntakeDown::IntakeDown(IntakeSubsystem* intakeSub) : intakeSubsystem(intakeSub) {
    AddRequirements(intakeSubsystem);
    SetName("IntakeDown");
}

// Called when the command is initially scheduled.
void IntakeDown::Initialize() {
    std::cout << "Inited IntakeDown\n";
    intakeSubsystem->PutIntakeDown();
}
