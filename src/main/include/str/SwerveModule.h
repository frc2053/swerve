// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/json.h>
#include <wpi/sendable/SendableBuilder.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include "Constants.h"
#include "Units.h"
#include "ctre/phoenix6/controls/PositionVoltage.hpp"

namespace str {

struct SwerveModuleConstants {
  const int driveMotorCanId;
  const int steerMotorCanId;
  const int steerEncoderCanId;
  const double steerEncoderOffset;
  const bool invertDrive;
  const bool invertSteer;
};

struct SteerCharData {
  units::volt_t motorVoltage{0_V};
  units::radian_t motorAngle{0_rad};
  units::radians_per_second_t motorVelocity{0_rad_per_s};
};

struct DriveCharData {
  units::volt_t motorVoltage{0_V};
  units::meter_t motorPosition{0_m};
  units::meters_per_second_t motorVelocity{0_mps};
};

class SwerveModule {
public:
  explicit SwerveModule(const SwerveModuleConstants& moduleConstants);

  void SimulationUpdate(units::meter_t driveDistance,
    units::meters_per_second_t driveVelocity, units::radian_t steerDistance,
    units::radians_per_second_t steerVelocity);

  DriveCharData GetDriveCharData();
  SteerCharData GetSteerCharData();

  frc::SwerveModulePosition GetPosition(bool refresh);
  frc::SwerveModulePosition GetCachedPosition() const;
  frc::SwerveModuleState GetState() const;
  void GoToState(
    const frc::SwerveModuleState& state, bool openLoop, bool optimize);

  void ResetPosition();

  void SetSteerVoltage(units::volt_t volts);
  void SetDriveVoltage(units::volt_t volts);

  void LockSteerAtZero();

  std::array<ctre::phoenix6::BaseStatusSignal*, 6> GetSignals();
  void OptimizeBusSignals();

  void Log(int moduleIndex);

  static units::meter_t ConvertMotorToWheelDistance(
    units::radian_t motorRotations);
  static units::meters_per_second_t ConvertMotorSpeedToWheelVelocity(
    units::radians_per_second_t motorVelocity);

  static units::radian_t ConvertWheelDistanceToMotorShaftRotations(
    units::meter_t wheelRotations);
  static units::radians_per_second_t ConvertWheelVelocityToMotorVelocity(
    units::meters_per_second_t wheelVelocity);

  static units::radian_t ConvertOutputShaftPositionToMotorShaftPosition(
    units::radian_t outputShaftPosition);
  static units::radians_per_second_t
  ConvertOutputShaftVelocityToMotorShaftVelocity(
    units::radians_per_second_t outputShaftVelocity);

private:
  ctre::phoenix::StatusCode ConfigureDriveMotor(bool invertDrive);
  ctre::phoenix::StatusCode ConfigureSteerMotor(bool invertSteer);
  ctre::phoenix::StatusCode ConfigureSteerEncoder(double encoderOffset);

  ctre::phoenix6::hardware::TalonFX driveMotor;
  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::CANcoder steerEncoder;

  ctre::phoenix6::StatusSignal<units::turn_t> steerAngleSignal
    = steerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>
    steerAngleVelocitySignal = steerMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> steerVoltageSignal
    = steerMotor.GetMotorVoltage();

  ctre::phoenix6::StatusSignal<units::turn_t> drivePositionSignal
    = driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocitySignal
    = driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> driveVoltageSignal
    = driveMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicVoltage steerAngleSetter{0_rad};
  ctre::phoenix6::controls::VoltageOut steerVoltageSetter{0_V};

  ctre::phoenix6::controls::VelocityTorqueCurrentFOC driveVelocitySetter{
    0_rad_per_s};
  ctre::phoenix6::controls::VoltageOut driveVoltageSetter{0_V};

  frc::SwerveModuleState currentState{};
  frc::SwerveModulePosition currentPosition{};

  constants::swerve::ModuleDriveGains currentDrivingGains{
    constants::swerve::driveGains};
  constants::swerve::ModuleSteerGains currentSteeringGains{
    constants::swerve::steerGains};

  units::radian_t currentAngleSetpoint{0};
  units::meters_per_second_t currentDriveSetpoint{0};

  // SIM STUFF
  ctre::phoenix6::sim::TalonFXSimState& simSteerMotor
    = steerMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& simDriveMotor
    = driveMotor.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState& simEncoder
    = steerEncoder.GetSimState();
};
} // namespace str
