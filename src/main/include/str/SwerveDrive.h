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

#include <memory>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "Constants.h"
#include "SwerveModule.h"

namespace str {
class SwerveDrive {
public:
  SwerveDrive();

  void UpdateOdometry();
  void Log();
  frc::Pose2d GetPose() const;
  std::array<frc::Pose2d, 4> GetModulePoses() const;

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
    constants::swerve::physical::moduleLocations[0],
    constants::swerve::physical::moduleLocations[1],
    constants::swerve::physical::moduleLocations[2],
    constants::swerve::physical::moduleLocations[3]};

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

  // Logging
  nt::NetworkTableInstance ntInst{nt::NetworkTableInstance::GetDefault()};
  std::shared_ptr<nt::NetworkTable> table{ntInst.GetTable("swerveInfo")};
  frc::Field2d ntField{};
};
} // namespace str
