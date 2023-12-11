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
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "str/Units.h"

DrivebaseSubsystem::DrivebaseSubsystem()
  : choreoController(choreolib::Choreo::ChoreoSwerveController(
    xTranslationController, yTranslationController, rotationController))
{
  LoadChoreoTrajectories();
}

bool DrivebaseSubsystem::HavePIDsChanged(units::scalar_t transP,
  units::scalar_t transI, units::scalar_t transD, units::scalar_t rotP,
  units::scalar_t rotI, units::scalar_t rotD)
{
  return !units::essentiallyEqual(
           units::scalar_t{xTranslationController.GetP()}, transP, 1e-6)
    || !units::essentiallyEqual(
      units::scalar_t{xTranslationController.GetI()}, transI, 1e-6)
    || !units::essentiallyEqual(
      units::scalar_t{xTranslationController.GetD()}, transD, 1e-6)
    || !units::essentiallyEqual(
      units::scalar_t{rotationController.GetP()}, rotP, 1e-6)
    || !units::essentiallyEqual(
      units::scalar_t{rotationController.GetI()}, rotI, 1e-6)
    || !units::essentiallyEqual(
      units::scalar_t{rotationController.GetD()}, rotD, 1e-6);
}

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic()
{
  if (pathTuning) {
    double newTransP = frc::SmartDashboard::GetNumber("Drivebase/TRANS_P", 0);
    double newTransI = frc::SmartDashboard::GetNumber("Drivebase/TRANS_I", 0);
    double newTransD = frc::SmartDashboard::GetNumber("Drivebase/TRANS_D", 0);
    double newRotP = frc::SmartDashboard::GetNumber("Drivebase/ROT_P", 0);
    double newRotI = frc::SmartDashboard::GetNumber("Drivebase/ROT_I", 0);
    double newRotD = frc::SmartDashboard::GetNumber("Drivebase/ROT_D", 0);
    if (HavePIDsChanged(
          newTransP, newTransI, newTransD, newRotP, newRotI, newRotD)) {
      SetTranslationPIDs(newTransP, newTransI, newTransD);
      SetRotationPIDs(newRotP, newRotI, newRotD);
    }
  }
  swerveDrive.Log();
}

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

void DrivebaseSubsystem::SetTranslationPIDs(double p, double i, double d)
{
  xTranslationController.SetPID(p, i, d);
  yTranslationController.SetPID(p, i, d);
}

void DrivebaseSubsystem::SetRotationPIDs(double p, double i, double d)
{
  rotationController.SetPID(p, i, d);
}

void DrivebaseSubsystem::SetPathTuning(bool onOff) { pathTuning = onOff; }

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
      choreolib::Choreo::ChoreoSwerveCommandPtr(
        pathMap[pathName()], [this] {
          return swerveDrive.GetPose();
        },
        choreoController,
        [this](frc::ChassisSpeeds speeds) {
          swerveDrive.SetChassisSpeeds(speeds, false);
        },
        true,
        {this}
      ),
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

frc2::CommandPtr DrivebaseSubsystem::TunePathPid() {
  // clang-foramt off
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      SetPathTuning(true);
      frc::SmartDashboard::PutNumber("Drivebase/TRANS_P", xTranslationController.GetP());
      frc::SmartDashboard::PutNumber("Drivebase/TRANS_I", xTranslationController.GetI());
      frc::SmartDashboard::PutNumber("Drivebase/TRANS_D", xTranslationController.GetD());
      frc::SmartDashboard::PutNumber("Drivebase/ROT_P", rotationController.GetP());
      frc::SmartDashboard::PutNumber("Drivebase/ROT_I", rotationController.GetI());
      frc::SmartDashboard::PutNumber("Drivebase/ROT_D", rotationController.GetD());
    })
  );
  // clang-foramt on
}

frc2::CommandPtr DrivebaseSubsystem::DoneTuningPathPids() {
  return frc2::cmd::RunOnce([this] {
    SetPathTuning(false);
  });
}
