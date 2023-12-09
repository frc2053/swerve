// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DrivebaseSubsystem.h"

#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <filesystem>

#include "choreo/lib/Choreo.h"
#include "frc/Filesystem.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc2/command/CommandPtr.h"

DrivebaseSubsystem::DrivebaseSubsystem()
  : choreoController(choreolib::Choreo::ChoreoSwerveController(
    frc::PIDController{constants::swerve::pathplanning::TRANSLATION_P,
      constants::swerve::pathplanning::TRANSLATION_I,
      constants::swerve::pathplanning::TRANSLATION_D},
    frc::PIDController{constants::swerve::pathplanning::TRANSLATION_P,
      constants::swerve::pathplanning::TRANSLATION_I,
      constants::swerve::pathplanning::TRANSLATION_D},
    frc::PIDController{constants::swerve::pathplanning::ROTATION_P,
      constants::swerve::pathplanning::ROTATION_I,
      constants::swerve::pathplanning::ROTATION_D}))
{
  LoadChoreoTrajectories();
}

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic() { swerveDrive.Log(); }

void DrivebaseSubsystem::SimulationPeriodic()
{
  swerveDrive.SimulationUpdate();
}

void DrivebaseSubsystem::UpdateOdometry() { swerveDrive.UpdateOdometry(); }

void DrivebaseSubsystem::LoadChoreoTrajectories()
{
  for (const auto& entry : std::filesystem::directory_iterator(
         frc::filesystem::GetDeployDirectory() + "/choreo/")) {
    std::string fileName = entry.path().stem().string();
    fmt::print("Loaded choreo trajectory: {}\n", fileName);
    pathMap[fileName] = choreolib::Choreo::GetTrajectory(fileName);
  }
}

frc2::CommandPtr DrivebaseSubsystem::ResetPosition(
  std::function<frc::Pose2d()> newPosition)
{
  return frc2::cmd::RunOnce([this, newPosition] {
    swerveDrive.TareEverything();
    swerveDrive.SeedFieldRelative(newPosition());
  });
}

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

frc2::CommandPtr DrivebaseSubsystem::SelfTest()
{
  return swerveDrive.SelfTest({this});
}

frc2::CommandPtr DrivebaseSubsystem::MeasureWheelDiam(
  std::function<bool()> done)
{
  return swerveDrive.MeasureWheelDiam(done, {this});
}

frc2::CommandPtr DrivebaseSubsystem::TuneSteerPID(std::function<bool()> done)
{
  return swerveDrive.TuneSteerPID(done, {this});
}

frc2::CommandPtr DrivebaseSubsystem::TuneDrivePID(std::function<bool()> done)
{
  return swerveDrive.TuneDrivePID(done, {this});
}

frc2::CommandPtr DrivebaseSubsystem::FollowChoreoTrajectory(
  std::function<std::string()> pathName)
{
  // clang-format off
  return frc2::cmd::Either(
    frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this, pathName] {
        swerveDrive.SeedFieldRelative(pathMap[pathName()].GetInitialPose());
        swerveDrive.GetField().GetObject("CurrentChoreoTrajectory")->SetPoses(pathMap[pathName()].GetPoses());
      }),
      choreolib::Choreo::ChoreoSwerveCommand(pathMap[pathName()], [this] {
        return swerveDrive.GetPose();
      },
      choreoController,
      [this](frc::ChassisSpeeds speeds) {
        swerveDrive.SetChassisSpeeds(speeds, false);
      },
      true,
      {this}),
      frc2::cmd::RunOnce([this] {
        swerveDrive.Drive(0_mps, 0_mps, 0_rad_per_s, false);
      })
    ),
    frc2::cmd::Print("ERROR: Choreo path wasn't found in pathMap!!!!\n\n\n\n"),
    [this, pathName] {
      return pathMap.contains(pathName());
    }
  );
  // clang-foramt on
}
