// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>

#include <functional>
#include <string>
#include <unordered_map>

#include "Constants.h"
#include "choreo/lib/Choreo.h"
#include "choreo/lib/ChoreoTrajectory.h"
#include "frc/controller/PIDController.h"
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
  frc2::CommandPtr ResetPosition(std::function<frc::Pose2d()> newPosition);

  frc2::CommandPtr DriveFactory(std::function<double()> fow,
    std::function<double()> side, std::function<double()> rot);
  frc2::CommandPtr TurnToAngleFactory(std::function<double()> fow,
    std::function<double()> side,
    std::function<frc::TrapezoidProfile<units::radians>::State()> angleProfile,
    std::function<bool()> wantsToOverride);

  frc2::CommandPtr CharacterizeSteerMotors(
    std::function<bool()> nextStepButton);
  frc2::CommandPtr CharacterizeDriveMotors(
    std::function<bool()> nextStepButton);
  frc2::CommandPtr SelfTest();
  frc2::CommandPtr MeasureWheelDiam(std::function<bool()> done);
  frc2::CommandPtr TuneSteerPID(std::function<bool()> done);
  frc2::CommandPtr TuneDrivePID(std::function<bool()> done);
  frc2::CommandPtr TunePathPid();
  frc2::CommandPtr DoneTuningPathPids();
  frc2::CommandPtr FollowChoreoTrajectory(
    std::function<std::string()> pathName);
  frc2::CommandPtr ZeroYawCMD();

  void SetTranslationPIDs(double p, double i, double d);
  void SetRotationPIDs(double p, double i, double d);
  void SetPathTuning(bool onOff);

private:
  str::SwerveDrive swerveDrive{};

  void LoadChoreoTrajectories();
  void SetupAutoBuilder();

  frc::PIDController xTranslationController{
    constants::swerve::pathplanning::TRANSLATION_P,
    constants::swerve::pathplanning::TRANSLATION_I,
    constants::swerve::pathplanning::TRANSLATION_D};
  frc::PIDController yTranslationController{
    constants::swerve::pathplanning::TRANSLATION_P,
    constants::swerve::pathplanning::TRANSLATION_I,
    constants::swerve::pathplanning::TRANSLATION_D};
  frc::PIDController rotationController{
    constants::swerve::pathplanning::ROTATION_P,
    constants::swerve::pathplanning::ROTATION_I,
    constants::swerve::pathplanning::ROTATION_D};

  choreolib::ChoreoControllerFunction choreoController;

  frc::ProfiledPIDController<units::radians> thetaController{
    constants::swerve::pathplanning::ROTATION_P,
    constants::swerve::pathplanning::ROTATION_I,
    constants::swerve::pathplanning::ROTATION_D,
    constants::swerve::pathplanning::GLOBAL_THETA_CONTROLLER_CONSTRAINTS};

  std::unordered_map<std::string, choreolib::ChoreoTrajectory> pathMap;
  bool pathTuning{false};
  bool HavePIDsChanged(units::scalar_t transP, units::scalar_t transI,
    units::scalar_t transD, units::scalar_t rotP, units::scalar_t rotI,
    units::scalar_t rotD);
};
