// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "str/SwerveDrive.h"

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void UpdateOdometry();

private:
  str::SwerveDrive swerveDrive{};
};
