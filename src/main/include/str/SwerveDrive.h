// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <units/current.h>

#include <array>
#include <functional>
#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include "Constants.h"
#include "SwerveDriveSim.h"
#include "SwerveModule.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "str/SwerveDriveSim.h"
#include "units/angular_velocity.h"

namespace str {
class SwerveDrive {
public:
  SwerveDrive();

  void UpdateOdometry();
  void Log();
  void SimulationUpdate();
  void TareEverything();
  void SeedFieldRelative();
  void SeedFieldRelative(const frc::Pose2d& location);

  void Drive(units::meters_per_second_t vx, units::meters_per_second_t vy,
    units::radians_per_second_t omega, bool openLoop);

  void SetChassisSpeeds(
    const frc::ChassisSpeeds& newChassisSpeeds, bool openLoop);

  void SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool openLoop,
    bool optimize = true);

  frc::Rotation2d GetHeading() const;
  frc::Rotation2d GetGyroYaw() const;
  frc::Pose2d GetPose() const;
  frc::Pose2d GetSimPose() const;
  units::ampere_t GetCurrentDraw() const;
  std::array<frc::Pose2d, 4> GetModulePoses() const;

  frc2::CommandPtr CharacterizeSteerMotors(
    std::function<bool()> nextStepButton, frc2::Requirements reqs);

  frc2::CommandPtr CharacterizeDriveMotors(
    std::function<bool()> nextStepButton, frc2::Requirements reqs);

  frc2::CommandPtr SelfTest(frc2::Requirements reqs);

  frc2::CommandPtr MeasureWheelDiam(
    std::function<bool()> done, frc2::Requirements reqs);

  frc2::CommandPtr TuneSteerPID(
    std::function<bool()> done, frc2::Requirements reqs);
  frc2::CommandPtr TuneDrivePID(
    std::function<bool()> done, frc2::Requirements reqs);

  frc::Field2d& GetField();

private:
  std::array<SwerveModule, 4> swerveModules
    = {SwerveModule{SwerveModuleConstants{constants::swerve::can::FL_DRIVE,
         constants::swerve::can::FL_STEER, constants::swerve::can::FL_ENC,
         constants::swerve::physical::FL_ENCODER_OFFSET, false, true}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::FR_DRIVE,
        constants::swerve::can::FR_STEER, constants::swerve::can::FR_ENC,
        constants::swerve::physical::FR_ENCODER_OFFSET, false, true}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::BL_DRIVE,
        constants::swerve::can::BL_STEER, constants::swerve::can::BL_ENC,
        constants::swerve::physical::BL_ENCODER_OFFSET, false, true}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::BR_DRIVE,
        constants::swerve::can::BR_STEER, constants::swerve::can::BR_ENC,
        constants::swerve::physical::BR_ENCODER_OFFSET, false, true}}};

  std::array<frc::SwerveModulePosition, 4> modulePositions{
    swerveModules[0].GetPosition(true),
    swerveModules[1].GetPosition(true),
    swerveModules[2].GetPosition(true),
    swerveModules[3].GetPosition(true),
  };

  frc::SwerveDrivePoseEstimator<4> poseEstimator{
    constants::swerve::physical::KINEMATICS, frc::Rotation2d{}, modulePositions,
    frc::Pose2d{}};

  ctre::phoenix6::hardware::Pigeon2 imu{constants::swerve::can::IMU, "*"};
  units::radian_t imuYaw{};
  units::radians_per_second_t imuRate{};
  units::radian_t fieldRelativeOffset{};

  std::array<ctre::phoenix6::BaseStatusSignal*, 26> allModuleSignals;

  // Simulation
  SwerveDriveSim swerveSim{};
  ctre::phoenix6::sim::Pigeon2SimState& imuSimState = imu.GetSimState();
  units::ampere_t totalCurrentDraw{0};
  std::array<frc::SwerveModulePosition, 4> lastPositions{};
  frc::Rotation2d lastAngle{};
  units::second_t lastUpdateTimestamp{frc::Timer::GetFPGATimestamp()};

  // Stats for fast update time
  units::second_t lastTime = 0_s;
  units::second_t currentTime = 0_s;
  units::second_t averageLoopTime = 0_s;

  // Logging
  frc::Field2d ntField{};

  // Characterization
  //  This is for characterization
  wpi::json flSteerModuleData{};
  wpi::json driveData{};

  units::volt_t quasistaticVolts = 0_V;
  static constexpr auto quasistaticStep{0.25_V / 1_s};
  units::volt_t dynamicStepVolts = 7_V;
};
} // namespace str
