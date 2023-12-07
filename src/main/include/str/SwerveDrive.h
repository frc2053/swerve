// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <units/current.h>

#include <array>
#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include "Constants.h"
#include "SwerveDriveSim.h"
#include "SwerveModule.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "str/SwerveDriveSim.h"

namespace str {
class SwerveDrive {
public:
  SwerveDrive();

  void UpdateOdometry();
  void Log();
  void SimulationUpdate();

  void Drive(units::meters_per_second_t vx, units::meters_per_second_t vy,
    units::radians_per_second_t omega, bool openLoop);

  void SetChassisSpeeds(
    const frc::ChassisSpeeds& newChassisSpeeds, bool openLoop);

  void SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool openLoop);

  frc::Rotation2d GetHeading() const;

  frc::Rotation2d GetGyroYaw() const;

  frc::Pose2d GetPose() const;
  frc::Pose2d GetSimPose() const;
  units::ampere_t GetCurrentDraw() const;
  std::array<frc::Pose2d, 4> GetModulePoses() const;

private:
  std::array<SwerveModule, 4> swerveModules
    = {SwerveModule{SwerveModuleConstants{constants::swerve::can::FL_DRIVE,
         constants::swerve::can::FL_STEER, constants::swerve::can::FL_ENC,
         constants::swerve::physical::FL_ENCODER_OFFSET, false, false}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::FR_DRIVE,
        constants::swerve::can::FR_STEER, constants::swerve::can::FR_ENC,
        constants::swerve::physical::FR_ENCODER_OFFSET, false, false}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::BL_DRIVE,
        constants::swerve::can::BL_STEER, constants::swerve::can::BL_ENC,
        constants::swerve::physical::BL_ENCODER_OFFSET, false, false}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::BR_DRIVE,
        constants::swerve::can::BR_STEER, constants::swerve::can::BR_ENC,
        constants::swerve::physical::BR_ENCODER_OFFSET, false, false}}};

  std::array<frc::SwerveModulePosition, 4> modulePostions{
    swerveModules[0].GetPosition(true),
    swerveModules[1].GetPosition(true),
    swerveModules[2].GetPosition(true),
    swerveModules[3].GetPosition(true),
  };

  frc::SwerveDrivePoseEstimator<4> poseEstimator{
    constants::swerve::physical::KINEMATICS, frc::Rotation2d{}, modulePostions,
    frc::Pose2d{}};

  ctre::phoenix6::hardware::Pigeon2 imu{constants::swerve::can::IMU, "*"};
  units::radian_t imuYaw{};

  std::array<ctre::phoenix6::BaseStatusSignal*, 26> allModuleSignals;

  // Simulation
  SwerveDriveSim swerveSim{};
  ctre::phoenix6::sim::Pigeon2SimState& imuSimState = imu.GetSimState();
  units::ampere_t totalCurrentDraw{0};
  std::array<frc::SwerveModulePosition, 4> lastPositions{};
  frc::Rotation2d lastAngle{};

  // Stats for fast update time
  units::second_t lastTime = 0_s;
  units::second_t currentTime = 0_s;
  units::second_t averageLoopTime = 0_s;

  // Logging
  nt::NetworkTableInstance ntInst{nt::NetworkTableInstance::GetDefault()};
  std::shared_ptr<nt::NetworkTable> table{ntInst.GetTable("swerveInfo")};
  frc::Field2d ntField{};
};
} // namespace str
