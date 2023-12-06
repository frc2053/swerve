// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDriveSim.h"

#include <frc/RobotController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/system/Discretization.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <units/current.h>
#include <units/impedance.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <algorithm>

#include <Eigen/src/Core/Matrix.h>

#include "Constants.h"
#include "str/SwerveModule.h"
#include "str/Units.h"

using namespace str;

using kv_inverse = units::inverse<units::kv_unit>;
using kv_inverse_t = units::unit_t<kv_inverse>;

SwerveDriveSim::SwerveDriveSim() = default;

void SwerveDriveSim::SetDriveInputs(const std::array<units::volt_t, 4>& inputs)
{
  units::volt_t battVoltage = frc::RobotController::GetBatteryVoltage();
  for (int i = 0; i < driveInputs.size(); i++) {
    driveInputs[i] = std::clamp(inputs[i], -battVoltage, battVoltage);
  }
}

void SwerveDriveSim::SetSteerInputs(const std::array<units::volt_t, 4>& inputs)
{
  units::volt_t battVoltage = frc::RobotController::GetBatteryVoltage();
  for (int i = 0; i < steerInputs.size(); i++) {
    steerInputs[i] = std::clamp(inputs[i], -battVoltage, battVoltage);
  }
}

Eigen::Matrix<double, 2, 1> SwerveDriveSim::CalculateX(
  const Eigen::Matrix<double, 2, 2>& discA,
  const Eigen::Matrix<double, 2, 1>& discB,
  const Eigen::Matrix<double, 2, 1>& x, units::volt_t input, units::volt_t kS)
{
  Eigen::Matrix<double, 2, 1> Ax = discA * x;
  units::meters_per_second_t nextStateVel{Ax(1, 0)};
  units::volt_t inputToStop{nextStateVel / kv_inverse_t{-discB(1, 0)}};
  units::volt_t ksSystemEffect = std::clamp(inputToStop, -kS, kS);

  nextStateVel += kv_inverse_t{discB(1, 0)} * ksSystemEffect;
  inputToStop = nextStateVel / kv_inverse_t{-discB(1, 0)};
  int signToStop = units::sgn(inputToStop);
  int inputSign = units::sgn(input);
  units::volt_t ksInputEffect = 0_V;

  if (units::math::abs(ksSystemEffect) < kS) {
    units::volt_t absInput = units::math::abs(input);
    ksInputEffect = -std::clamp(kS * inputSign, -absInput, absInput);
  } else if ((input * signToStop) > (inputToStop * signToStop)) {
    units::volt_t absInput = units::math::abs(input - inputToStop);
    ksInputEffect = -std::clamp(kS * inputSign, -absInput, absInput);
  }

  Eigen::Matrix<double, 2, 1> Bu = discB
    * Eigen::Matrix<double, 1, 1>{
      (input + ksSystemEffect + ksInputEffect).value()};
  return Ax + Bu;
}

void SwerveDriveSim::Update(units::second_t dt)
{
  Eigen::Matrix<double, 2, 2> discADrive{};
  Eigen::Matrix<double, 2, 1> discBDrive{};
  frc::DiscretizeAB<2, 1>(
    drivePlant.A(), drivePlant.B(), dt, &discADrive, &discBDrive);

  Eigen::Matrix<double, 2, 2> discASteer{};
  Eigen::Matrix<double, 2, 1> discBSteer{};
  frc::DiscretizeAB<2, 1>(
    steerPlant.A(), steerPlant.B(), dt, &discASteer, &discBSteer);

  std::array<frc::SwerveModulePosition, 4> moduleDeltas;
  for (int i = 0; i < 4; i++) {
    units::meter_t prevDriveStatePos = units::meter_t{driveStates[i](0, 0)};
    driveStates[i] = CalculateX(discADrive, discBDrive, driveStates[i],
      driveInputs[i], constants::swerve::driveGains.kS);
    units::meter_t currentDriveStatePos = units::meter_t{driveStates[i](0, 0)};
    steerStates[i] = CalculateX(discASteer, discBSteer, steerStates[i],
      steerInputs[i], constants::swerve::steerGains.kS);
    units::radian_t currentSteerStatePos
      = units::radian_t{steerStates[i](0, 0)};
    moduleDeltas[i]
      = frc::SwerveModulePosition{currentDriveStatePos - prevDriveStatePos,
        frc::Rotation2d{currentSteerStatePos}};
  }

  frc::Twist2d twist
    = constants::swerve::physical::KINEMATICS.ToTwist2d(moduleDeltas);
  pose = pose.Exp(twist);
  omega = twist.dtheta / dt;
}

void SwerveDriveSim::Reset(const frc::Pose2d& newPose, bool preserveMotion)
{
  pose = newPose;
  if (!preserveMotion) {
    for (int i = 0; i < 4; i++) {
      driveStates[i] = Eigen::Vector2d{0, 0};
      steerStates[i] = Eigen::Vector2d{0, 0};
    }
    omega = 0_rad_per_s;
  }
}

void SwerveDriveSim::Reset(const frc::Pose2d& newPose,
  const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleDriveStates,
  const std::array<Eigen::Matrix<double, 2, 1>, 4>& moduleSteerStates)
{
  pose = newPose;
  for (int i = 0; i < 4; i++) {
    driveStates[i] = moduleDriveStates[i];
    steerStates[i] = moduleSteerStates[i];
  }
  omega
    = constants::swerve::physical::KINEMATICS.ToChassisSpeeds(GetModuleStates())
        .omega;
}

frc::Pose2d SwerveDriveSim::GetPose() const { return pose; }

std::array<frc::SwerveModulePosition, 4>
SwerveDriveSim::GetModulePositions() const
{
  std::array<frc::SwerveModulePosition, 4> positions;
  for (int i = 0; i < 4; i++) {
    positions[i]
      = frc::SwerveModulePosition{units::meter_t{driveStates[i](0, 0)},
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}}};
  }
  return positions;
}

std::array<frc::SwerveModuleState, 4> SwerveDriveSim::GetModuleStates() const
{
  std::array<frc::SwerveModuleState, 4> states;
  for (int i = 0; i < 4; i++) {
    states[i]
      = frc::SwerveModuleState{units::meters_per_second_t{driveStates[i](1, 0)},
        frc::Rotation2d{units::radian_t{steerStates[i](0, 0)}}};
  }
  return states;
}

std::array<Eigen::Matrix<double, 2, 1>, 4>
SwerveDriveSim::GetDriveStates() const
{
  return driveStates;
}

std::array<Eigen::Matrix<double, 2, 1>, 4>
SwerveDriveSim::GetSteerStates() const
{
  return steerStates;
}

units::radians_per_second_t SwerveDriveSim::GetOmega() const { return omega; }

units::ampere_t SwerveDriveSim::GetCurrentDraw(
  units::radians_per_second_t velocityOfMotor, units::volt_t input,
  units::volt_t battery, bool isFOC)
{
  frc::DCMotor::radians_per_second_per_volt_t kV{0};
  units::ohm_t r{0};
  if (isFOC) {
    kV = constants::swerve::physical::SWERVE_MOTOR_FOC.Kv;
    r = constants::swerve::physical::SWERVE_MOTOR_FOC.R;
  } else {
    kV = constants::swerve::physical::SWERVE_MOTOR.Kv;
    r = constants::swerve::physical::SWERVE_MOTOR.R;
  }
  units::volt_t effVolts = input - velocityOfMotor / kV;
  if (input >= 0_V) {
    effVolts = std::clamp(effVolts, 0_V, input);
  } else {
    effVolts = std::clamp(effVolts, input, 0_V);
  }
  return (input / battery) * (effVolts / r);
}

std::array<units::ampere_t, 4> SwerveDriveSim::GetDriveCurrentDraw(
  bool isFOC) const
{
  std::array<units::ampere_t, 4> currents;
  for (int i = 0; i < 4; i++) {
    units::radians_per_second_t motorSpeed
      = SwerveModule::ConvertWheelVelocityToMotorVelocity(
        units::meters_per_second_t{driveStates[i](1, 0)});
    currents[i] = GetCurrentDraw(motorSpeed, driveInputs[i],
      frc::RobotController::GetBatteryVoltage(), isFOC);
  }
  return currents;
}

std::array<units::ampere_t, 4> SwerveDriveSim::GetSteerCurrentDraw(
  bool isFOC) const
{
  std::array<units::ampere_t, 4> currents;
  for (int i = 0; i < 4; i++) {
    units::radians_per_second_t motorSpeed
      = SwerveModule::ConvertOutputShaftVelocityToMotorShaftVelocity(
        units::radians_per_second_t{steerStates[i](1, 0)});
    currents[i] = GetCurrentDraw(motorSpeed, steerInputs[i],
      frc::RobotController::GetBatteryVoltage(), isFOC);
  }
  return currents;
}

units::ampere_t SwerveDriveSim::GetTotalCurrentDraw(bool isFOC) const
{
  units::ampere_t sum{0};
  for (const units::ampere_t& val : GetDriveCurrentDraw(isFOC)) {
    sum = sum + val;
  }
  for (const units::ampere_t& val : GetSteerCurrentDraw(isFOC)) {
    sum = sum + val;
  }
  return sum;
}
