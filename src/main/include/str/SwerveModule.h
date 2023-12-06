// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/DCMotorSim.h>
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

#include "Units.h"

namespace str {

struct ModuleGains {
  units::ka_unit_t kA{0};
  units::kv_unit_t kV{0};
  units::volt_t kS{0};
  units::scalar_t kP{0};
  units::scalar_t kI{0};
  units::scalar_t kD{0};
};

struct SwerveModuleConstants {
  const int driveMotorCanId;
  const int steerMotorCanId;
  const int steerEncoderCanId;
  const double steerEncoderOffset;
  const bool invertDrive;
  const bool invertSteer;
  const ModuleGains driveGains;
  const ModuleGains steerGains;
  const units::scalar_t driveGearing;
  const units::scalar_t steerGearing;
  const units::ampere_t driveSlipCurrent;
  const units::radians_per_second_t steerMotionMagicVel;
  const units::radians_per_second_squared_t steerMotionMagicAccel;
  const units::meter_t wheelRadius;
  const units::scalar_t driveSteerCoupling;
  const units::meters_per_second_t maxDriveSpeed;
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
    units::meters_per_second_t driveVelocity, units::ampere_t driveCurrent,
    units::radian_t steerDistance, units::radians_per_second_t steerVelocity,
    units::ampere_t steerCurrent);

  DriveCharData GetDriveCharData();
  SteerCharData GetSteerCharData();

  frc::SwerveModulePosition GetPosition(bool refresh);
  frc::SwerveModulePosition GetCachedPosition() const;
  frc::SwerveModuleState GetState() const;
  void GoToState(const frc::SwerveModuleState& state, bool openLoop);

  void ResetPosition();

  void SetSteerVoltage(units::volt_t volts);
  void SetDriveVoltage(units::volt_t volts);

  std::array<ctre::phoenix6::BaseStatusSignal*, 6> GetSignals();
  void OptimizeBusSignals();

private:
  ctre::phoenix::StatusCode ConfigureDriveMotor(bool invertDrive,
    units::scalar_t driveGearing, units::ampere_t slipCurrent);
  ctre::phoenix::StatusCode ConfigureSteerMotor(bool invertSteer,
    units::scalar_t steerGearing,
    const units::radians_per_second_t steerMotionMagicVel,
    const units::radians_per_second_squared_t steerMotionMagicAccel);
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

  units::meter_t ConvertOutputShaftToWheelDistance(
    units::radian_t shaftRotations) const;
  units::meters_per_second_t ConvertOutputShaftToWheelVelocity(
    units::radians_per_second_t shaftVelocity) const;

  units::radian_t ConvertWheelDistanceToMotorShaftRotations(
    units::meter_t wheelRotations) const;
  units::radians_per_second_t ConvertWheelVelocityToMotorVelocity(
    units::meters_per_second_t wheelVelocity) const;

  units::radian_t ConvertOutputShaftPositionToMotorShaftPosition(
    units::radian_t outputShaftPosition) const;
  units::radians_per_second_t ConvertOutputShaftVelocityToMotorShaftVelocity(
    units::radians_per_second_t outputShaftVelocity) const;

  ModuleGains currentDrivingGains;
  ModuleGains currentSteeringGains;

  const units::meter_t wheelRadius;
  const units::scalar_t steerGearing;
  const units::scalar_t driveGearing;
  const units::scalar_t driveSteerCoupling;
  const units::meters_per_second_t maxDriveSpeed;

  // SIM STUFF
  ctre::phoenix6::sim::TalonFXSimState& simSteerMotor
    = steerMotor.GetSimState();
  ctre::phoenix6::sim::TalonFXSimState& simDriveMotor
    = steerMotor.GetSimState();
  ctre::phoenix6::sim::CANcoderSimState& simEncoder
    = steerEncoder.GetSimState();
};
} // namespace str
