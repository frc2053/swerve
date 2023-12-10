// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "auto/Autos.h"
#include "subsystems/DrivebaseSubsystem.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  DrivebaseSubsystem& GetDrivebaseSubsystem();

private:
  void ConfigureBindings();

  frc2::CommandXboxController driverController{0};

  DrivebaseSubsystem driveSub;

  autos::Autos autos{driveSub};

  frc2::CommandPtr characterizeSteerCmd = driveSub.CharacterizeSteerMotors([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr characterizeDriveCmd = driveSub.CharacterizeDriveMotors([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr selfTestCmd = driveSub.SelfTest();
  frc2::CommandPtr measureWheelCmd = driveSub.MeasureWheelDiam([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneSteerCmd = driveSub.TuneSteerPID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneDriveCmd = driveSub.TuneDrivePID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr resetPositionCmd
    = driveSub.ResetPosition([] { return frc::Pose2d{}; });
  frc2::CommandPtr tunePathPidCmd = driveSub.TunePathPid();
  frc2::CommandPtr donePathTuningCmd = driveSub.DoneTuningPathPids();
};
