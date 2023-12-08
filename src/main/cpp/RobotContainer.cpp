// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings()
{
  driveSub.SetDefaultCommand(driveSub.DriveFactory(
    [this] {
      double fwdCmd
        = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
      return std::abs(fwdCmd) * fwdCmd;
    },
    [this] {
      double sideCmd
        = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
      return std::abs(sideCmd) * sideCmd;
    },
    [this] {
      double rotCmd
        = frc::ApplyDeadband<double>(-driverController.GetRightX(), 0.2);
      return std::abs(rotCmd) * rotCmd;
    }));

  driverController.A().OnTrue(driveSub.CharacterizeSteerMotors(
    [this] { return driverController.GetStartButtonPressed(); }));
  driverController.B().OnTrue(driveSub.CharacterizeDriveMotors(
    [this] { return driverController.GetStartButtonPressed(); }));
  driverController.Y().OnTrue(driveSub.SelfTest());
  driverController.X().OnTrue(driveSub.MeasureWheelDiam(
    [this] { return driverController.GetStartButton(); }));
  driverController.LeftBumper().OnTrue(driveSub.TuneSteerPID(
    [this] { return driverController.GetStartButtonPressed(); }));
  driverController.RightBumper().OnTrue(driveSub.TuneDrivePID(
    [this] { return driverController.GetStartButtonPressed(); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return frc2::cmd::Print("No autonomous command configured");
}

DrivebaseSubsystem& RobotContainer::GetDrivebaseSubsystem() { return driveSub; }
