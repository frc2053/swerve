// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>

#include <functional>

#include "str/SwerveDrive.h"

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  void UpdateOdometry();

  frc2::CommandPtr DriveFactory(std::function<double()> fow,
    std::function<double()> side, std::function<double()> rot);

  frc2::CommandPtr CharacterizeSteerMotors(
    std::function<bool()> nextStepButton);
  frc2::CommandPtr CharacterizeDriveMotors(
    std::function<bool()> nextStepButton);

private:
  str::SwerveDrive swerveDrive{};
};
