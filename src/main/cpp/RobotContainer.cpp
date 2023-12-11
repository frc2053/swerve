// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "frc/smartdashboard/SmartDashboard.h"

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

  driverController.Y().OnTrue(driveSub.TurnToAngleFactory(
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
      return frc::TrapezoidProfile<units::radians>::State{0_deg, 0_deg_per_s};
    },
    [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  driverController.X().OnTrue(driveSub.TurnToAngleFactory(
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
      return frc::TrapezoidProfile<units::radians>::State{90_deg, 0_deg_per_s};
    },
    [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  driverController.B().OnTrue(driveSub.TurnToAngleFactory(
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
      return frc::TrapezoidProfile<units::radians>::State{-90_deg, 0_deg_per_s};
    },
    [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  driverController.A().OnTrue(driveSub.TurnToAngleFactory(
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
      return frc::TrapezoidProfile<units::radians>::State{180_deg, 0_deg_per_s};
    },
    [this] { return std::abs(driverController.GetRightX()) > 0.2; }));

  frc::SmartDashboard::PutBoolean("Drivebase/DoneWithStep", false);

  frc::SmartDashboard::PutData(
    "Drivebase/CharacterizeSteerMotorsCmd", characterizeSteerCmd.get());

  frc::SmartDashboard::PutData(
    "Drivebase/CharacterizeDriveMotorsCmd", characterizeDriveCmd.get());

  frc::SmartDashboard::PutData("Drivebase/SelfTestCmd", selfTestCmd.get());

  frc::SmartDashboard::PutData(
    "Drivebase/MeasureWheelCmd", measureWheelCmd.get());

  frc::SmartDashboard::PutData("Drivebase/TuneSteerCmd", tuneSteerCmd.get());

  frc::SmartDashboard::PutData("Drivebase/TuneDriveCmd", tuneDriveCmd.get());

  frc::SmartDashboard::PutData(
    "Drivebase/ResetPosition", resetPositionCmd.get());

  frc::SmartDashboard::PutData("Drivebase/PathTuningCmd", tunePathPidCmd.get());

  frc::SmartDashboard::PutData(
    "Drivebase/DonePathTuningCmd", donePathTuningCmd.get());
}

frc2::Command* RobotContainer::GetAutonomousCommand()
{
  return autos.GetSelectedAutoCmd.get();
}

DrivebaseSubsystem& RobotContainer::GetDrivebaseSubsystem() { return driveSub; }
