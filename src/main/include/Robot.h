// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <optional>

#include "Constants.h"
#include "RobotContainer.h"
#include "str/SwerveModule.h"
#include "units/angular_acceleration.h"

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;
  void SimulationPeriodic() override;

private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  str::ModuleGains flDriveGains{
    str::ka_unit_t{0}, str::kv_unit_t{0}, 0_V, 3, 0, 0};
  str::ModuleGains flSteerGains{
    str::ka_unit_t{0}, str::kv_unit_t{0}, 0_V, 1, 0, 0};

  str::SwerveModuleConstants flConsts{1, 2, 3, 0, false, false, flDriveGains,
    flSteerGains, constants::swerve::physical::DRIVE_GEARING,
    constants::swerve::physical::STEER_GEARING, 400_A, 10_rad_per_s,
    units::radians_per_second_squared_t{100}, 4_in, 3.5,
    constants::swerve::physical::MAX_LINEAR_SPEED};

  str::SwerveModule flModule{flConsts};

  RobotContainer m_container;
};
