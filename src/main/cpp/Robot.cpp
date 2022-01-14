// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

void Robot::RobotInit() {
    SetNetworkTablesFlushEnabled(true);
}

void Robot::RobotPeriodic() {
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }
}

void Robot::TeleopPeriodic() {}
void Robot::TestPeriodic() {}

void Robot::SimulationPeriodic() {
    auto dt_current = m_container.GetRobotDriveSubsystem().GetCurrentDraw();
    auto shooter_current = m_container.GetShooterSubsystem().GetCurrentDraw();
    auto loadedVoltage = frc::sim::BatterySim::Calculate({dt_current/*, shooter_current*/});
    frc::sim::RoboRioSim::SetVInVoltage(loadedVoltage);
}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
