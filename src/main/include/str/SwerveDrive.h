// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "Constants.h"
#include "SwerveModule.h"

namespace str {
class SwerveDrive {
public:
  SwerveDrive();

  void UpdateOdometry();
  void Log();

private:
  const ModuleGains driveGains{
    units::ka_unit_t{0}, units::kv_unit_t{0}, 0_V, 3.0, 0.0, 0.0};

  const ModuleGains steerGains{
    units::ka_unit_t{0}, units::kv_unit_t{0}, 0_V, 1.0, 0.0, 0.0};

  std::array<SwerveModule, 4> swerveModules
    = {SwerveModule{SwerveModuleConstants{constants::swerve::can::FL_DRIVE,
         constants::swerve::can::FL_STEER, constants::swerve::can::FL_ENC,
         constants::swerve::physical::FL_ENCODER_OFFSET, false, false,
         driveGains, steerGains, constants::swerve::physical::DRIVE_GEARING,
         constants::swerve::physical::STEER_GEARING,
         constants::swerve::physical::SLIP_CURRENT,
         constants::swerve::physical::STEER_MOTION_MAGIC_VEL,
         constants::swerve::physical::STEER_MOTION_MAGIC_ACCEL,
         constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2,
         constants::swerve::physical::DRIVE_STEER_COUPLING,
         constants::swerve::physical::MAX_LINEAR_SPEED}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::FR_DRIVE,
        constants::swerve::can::FR_STEER, constants::swerve::can::FR_ENC,
        constants::swerve::physical::FR_ENCODER_OFFSET, false, false,
        driveGains, steerGains, constants::swerve::physical::DRIVE_GEARING,
        constants::swerve::physical::STEER_GEARING,
        constants::swerve::physical::SLIP_CURRENT,
        constants::swerve::physical::STEER_MOTION_MAGIC_VEL,
        constants::swerve::physical::STEER_MOTION_MAGIC_ACCEL,
        constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2,
        constants::swerve::physical::DRIVE_STEER_COUPLING,
        constants::swerve::physical::MAX_LINEAR_SPEED}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::BL_DRIVE,
        constants::swerve::can::BL_STEER, constants::swerve::can::BL_ENC,
        constants::swerve::physical::BL_ENCODER_OFFSET, false, false,
        driveGains, steerGains, constants::swerve::physical::DRIVE_GEARING,
        constants::swerve::physical::STEER_GEARING,
        constants::swerve::physical::SLIP_CURRENT,
        constants::swerve::physical::STEER_MOTION_MAGIC_VEL,
        constants::swerve::physical::STEER_MOTION_MAGIC_ACCEL,
        constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2,
        constants::swerve::physical::DRIVE_STEER_COUPLING,
        constants::swerve::physical::MAX_LINEAR_SPEED}},
      SwerveModule{SwerveModuleConstants{constants::swerve::can::BR_DRIVE,
        constants::swerve::can::BR_STEER, constants::swerve::can::BR_ENC,
        constants::swerve::physical::BR_ENCODER_OFFSET, false, false,
        driveGains, steerGains, constants::swerve::physical::DRIVE_GEARING,
        constants::swerve::physical::STEER_GEARING,
        constants::swerve::physical::SLIP_CURRENT,
        constants::swerve::physical::STEER_MOTION_MAGIC_VEL,
        constants::swerve::physical::STEER_MOTION_MAGIC_ACCEL,
        constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2,
        constants::swerve::physical::DRIVE_STEER_COUPLING,
        constants::swerve::physical::MAX_LINEAR_SPEED}}};

  frc::SwerveDriveKinematics<4> kinematics{
    constants::swerve::physical::FL_LOCATION,
    constants::swerve::physical::FR_LOCATION,
    constants::swerve::physical::BL_LOCATION,
    constants::swerve::physical::BR_LOCATION};

  std::array<frc::SwerveModulePosition, 4> modulePostions{
    swerveModules[0].GetPosition(true),
    swerveModules[1].GetPosition(true),
    swerveModules[2].GetPosition(true),
    swerveModules[3].GetPosition(true),
  };

  frc::SwerveDrivePoseEstimator<4> poseEstimator{
    kinematics, frc::Rotation2d{}, modulePostions, frc::Pose2d{}};

  ctre::phoenix6::hardware::Pigeon2 imu{constants::swerve::can::IMU, "*"};

  std::array<ctre::phoenix6::BaseStatusSignal*, 18> allModuleSignals;

  // Stats for fast update time
  units::second_t lastTime = 0_s;
  units::second_t currentTime = 0_s;
  units::second_t averageLoopTime = 0_s;
};
} // namespace str
