// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/DrivebaseSubsystem.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  DrivebaseSubsystem& GetDrivebaseSubsystem();

private:
  void ConfigureBindings();

  frc2::CommandXboxController driverController{0};

  DrivebaseSubsystem driveSub;
};
