// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/EigenCore.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/system/LinearSystem.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

#include <array>

#include "Constants.h"

namespace str {
class SwerveDriveSim {
public:
  SwerveDriveSim();
  void SetDriveInputs(const std::array<units::volt_t, 4>& inputs);
  void SetSteerInputs(const std::array<units::volt_t, 4>& inputs);
  void Update(units::second_t dt);
  void Reset(const frc::Pose2d& newPose, bool preserveMotion);
  void Reset(const frc::Pose2d& newPose,
    const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleDriveStates,
    const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleSteerStates);
  frc::Pose2d GetPose() const;
  std::array<frc::SwerveModulePosition, 4> GetModulePositions() const;
  std::array<frc::SwerveModuleState, 4> GetModuleStates() const;
  std::array<Eigen::Matrix<double, 2, 1>, 4> GetDriveStates() const;
  std::array<Eigen::Matrix<double, 2, 1>, 4> GetSteerStates() const;
  units::radians_per_second_t GetOmega() const;
  std::array<units::ampere_t, 4> GetDriveCurrentDraw(bool isFOC) const;
  std::array<units::ampere_t, 4> GetSteerCurrentDraw(bool isFOC) const;
  units::ampere_t GetTotalCurrentDraw(bool isFOC) const;

private:
  static Eigen::Matrix<double, 2, 1> CalculateX(
    const Eigen::Matrix<double, 2, 2>& discA,
    const Eigen::Matrix<double, 2, 1>& discB,
    const Eigen::Matrix<double, 2, 1>& x, units::volt_t input,
    units::volt_t kS);
  static units::ampere_t GetCurrentDraw(
    units::radians_per_second_t velocityOfMotor, units::volt_t input,
    units::volt_t battery, bool isFOC);
  frc::LinearSystem<2, 1, 2> drivePlant{
    (Eigen::MatrixXd(2, 2) << 0.0, 1.0, 0.0,
      (-constants::swerve::driveGains.kV / constants::swerve::driveGains.kA)
        .value())
      .finished(),
    (Eigen::MatrixXd(2, 1) << 0.0,
      (1.0 / constants::swerve::driveGains.kA).value())
      .finished(),
    (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 0.0, 1.0).finished(),
    (Eigen::MatrixXd(2, 1) << 0.0, 0.0).finished()};

  frc::LinearSystem<2, 1, 2> steerPlant{
    (Eigen::MatrixXd(2, 2) << 0.0, 1.0, 0.0,
      (-constants::swerve::steerGains.kV / constants::swerve::steerGains.kA)
        .value())
      .finished(),
    (Eigen::MatrixXd(2, 1) << 0.0,
      (1.0 / constants::swerve::steerGains.kA).value())
      .finished(),
    (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 0.0, 1.0).finished(),
    (Eigen::MatrixXd(2, 1) << 0.0, 0.0).finished()};

  std::array<units::volt_t, 4> driveInputs{};
  std::array<units::volt_t, 4> steerInputs{};
  std::array<Eigen::Matrix<double, 2, 1>, 4> driveStates{};
  std::array<Eigen::Matrix<double, 2, 1>, 4> steerStates{};
  frc::Pose2d pose{};
  units::radians_per_second_t omega{};
};
} // namespace str
