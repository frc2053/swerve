// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DrivebaseSubsystem.h"

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

DrivebaseSubsystem::DrivebaseSubsystem() = default;

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic() { swerveDrive.Log(); }

void DrivebaseSubsystem::SimulationPeriodic()
{
  swerveDrive.SimulationUpdate();
}

void DrivebaseSubsystem::UpdateOdometry() { swerveDrive.UpdateOdometry(); }

frc2::CommandPtr DrivebaseSubsystem::DriveFactory(std::function<double()> fow,
  std::function<double()> side, std::function<double()> rot)
{
  return frc2::RunCommand(
    [this, fow, side, rot]() {
      swerveDrive.Drive(fow() * constants::swerve::physical::MAX_LINEAR_SPEED,
        side() * constants::swerve::physical::MAX_LINEAR_SPEED,
        rot() * constants::swerve::physical::MAX_ROTATION_SPEED, true);
    },
    {this})
    .ToPtr()
    .WithName("Drive Factory");
}

frc2::CommandPtr DrivebaseSubsystem::CharacterizeSteerMotors(
  std::function<bool()> nextStepButton)
{
  return swerveDrive.CharacterizeSteerMotors(nextStepButton, {this});
}
frc2::CommandPtr DrivebaseSubsystem::CharacterizeDriveMotors(
  std::function<bool()> nextStepButton)
{
  return swerveDrive.CharacterizeDriveMotors(nextStepButton, {this});
}
