// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveModule.h"

#include <ctre/phoenix/StatusCodes.h>
#include <frc/DataLogManager.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

using namespace str;

SwerveModule::SwerveModule(const SwerveModuleConstants& moduleConstants)
  : driveMotor(moduleConstants.driveMotorCanId, "*")
  , steerMotor(moduleConstants.steerMotorCanId, "*")
  , steerEncoder(moduleConstants.steerEncoderCanId, "*")
  , currentDrivingGains(moduleConstants.driveGains)
  , currentSteeringGains(moduleConstants.steerGains)
  , wheelRadius(moduleConstants.wheelRadius)
  , steerGearing(moduleConstants.steerGearing)
  , driveGearing(moduleConstants.driveGearing)
  , driveSteerCoupling(moduleConstants.driveSteerCoupling)
  , maxDriveSpeed(moduleConstants.maxDriveSpeed)
{
  ctre::phoenix::StatusCode driveConfigCode
    = ConfigureDriveMotor(moduleConstants.invertDrive,
      moduleConstants.driveGearing, moduleConstants.driveSlipCurrent);
  if (!driveConfigCode.IsOK()) {
    frc::DataLogManager::Log(
      fmt::format("Swerve Module ConfigureDriveMotor wasn't ok it was: {}. "
                  "More info: {}",
        driveConfigCode.GetName(), driveConfigCode.GetDescription()));
  }
  ctre::phoenix::StatusCode steerConfigCode = ConfigureSteerMotor(
    moduleConstants.invertSteer, moduleConstants.steerGearing,
    moduleConstants.steerMotionMagicVel, moduleConstants.steerMotionMagicAccel);
  if (!steerConfigCode.IsOK()) {
    frc::DataLogManager::Log(
      fmt::format("Swerve Module ConfigureSteerMotor wasn't ok it was: {}. "
                  "More info: {}",
        steerConfigCode.GetName(), steerConfigCode.GetDescription()));
  }
  ctre::phoenix::StatusCode steerEncoderCode
    = ConfigureSteerEncoder(moduleConstants.steerEncoderOffset);
  if (!steerEncoderCode.IsOK()) {
    frc::DataLogManager::Log(
      fmt::format("Swerve Module ConfigureSteerEncoder wasn't ok it was: {}. "
                  "More info: {}",
        steerEncoderCode.GetName(), steerEncoderCode.GetDescription()));
  }
}

void SwerveModule::SimulationUpdate(units::meter_t driveDistance,
  units::meters_per_second_t driveVelocity, units::ampere_t driveCurrent,
  units::radian_t steerDistance, units::radians_per_second_t steerVelocity,
  units::ampere_t steerCurrent)
{
  simDriveMotor.SetRawRotorPosition(
    ConvertWheelDistanceToMotorShaftRotations(driveDistance));
  simDriveMotor.SetRotorVelocity(
    ConvertWheelVelocityToMotorVelocity(driveVelocity));
  // TODO: Might need to set supply voltage
  // simDriveMotor.SetSupplyVoltage();

  simSteerMotor.SetRawRotorPosition(
    ConvertOutputShaftPositionToMotorShaftPosition(steerDistance));
  simSteerMotor.SetRotorVelocity(
    ConvertOutputShaftVelocityToMotorShaftVelocity(steerVelocity));
  // TODO: Might need to set supply voltage
  // simSteerMotor.SetSupplyVoltage();

  simEncoder.SetRawPosition(steerDistance);
  simEncoder.SetVelocity(steerVelocity);
  // TODO: Might need to set supply voltage
  // simEncoder.SetSupplyVoltage();
}

DriveCharData SwerveModule::GetDriveCharData()
{
  ctre::phoenix6::BaseStatusSignal::WaitForAll(
    0_s, drivePositionSignal, driveVelocitySignal, driveVoltageSignal);

  return DriveCharData{driveVoltageSignal.GetValue(),
    ConvertOutputShaftToWheelDistance(drivePositionSignal.GetValue()),
    ConvertOutputShaftToWheelVelocity(driveVelocitySignal.GetValue())};
}

SteerCharData SwerveModule::GetSteerCharData()
{
  ctre::phoenix6::BaseStatusSignal::WaitForAll(
    0_s, steerAngleSignal, steerAngleVelocitySignal, steerVoltageSignal);

  return SteerCharData{steerVoltageSignal.GetValue(),
    steerAngleSignal.GetValue(), steerAngleVelocitySignal.GetValue()};
}

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh)
{
  if (refresh) {
    ctre::phoenix::StatusCode status
      = ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, steerAngleSignal,
        steerAngleVelocitySignal, steerVoltageSignal, drivePositionSignal,
        driveVelocitySignal, driveVoltageSignal);
    if (!status.IsOK()) {
      frc::DataLogManager::Log(
        fmt::format("Swerve Module GetPosition WaitForAll() status wasn't ok "
                    "it was: {}. More Info: {}",
          status.GetName(), status.GetDescription()));
    }
  }

  units::radian_t drivePosition
    = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      drivePositionSignal, driveVelocitySignal);

  units::radian_t steerPosition
    = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      steerAngleSignal, steerAngleVelocitySignal);

  drivePosition = drivePosition - (steerPosition * driveSteerCoupling);

  currentPosition = frc::SwerveModulePosition{
    ConvertOutputShaftToWheelDistance(drivePosition),
    frc::Rotation2d{steerPosition}};

  currentState = frc::SwerveModuleState{
    ConvertOutputShaftToWheelVelocity(driveVelocitySignal.GetValue()),
    frc::Rotation2d{steerPosition}};

  return currentPosition;
}

frc::SwerveModulePosition SwerveModule::GetCachedPosition() const
{
  return currentPosition;
}

frc::SwerveModuleState SwerveModule::GetState() const { return currentState; }

void SwerveModule::GoToState(const frc::SwerveModuleState& state, bool openLoop)
{
  frc::SwerveModuleState optimizedState
    = frc::SwerveModuleState::Optimize(state, currentState.angle);

  units::radians_per_second_t moduleTurnSpeed
    = steerAngleVelocitySignal.GetValue();
  units::radians_per_second_t driveRateBackout
    = moduleTurnSpeed * driveSteerCoupling;

  units::radians_per_second_t velocityToGoTo
    = ConvertWheelVelocityToMotorVelocity(optimizedState.speed)
    - driveRateBackout;

  steerMotor.SetControl(
    steerAngleSetter.WithPosition(optimizedState.angle.Radians()));
  if (openLoop) {
    driveMotor.SetControl(driveVoltageSetter.WithOutput(
      (velocityToGoTo / ConvertWheelVelocityToMotorVelocity(maxDriveSpeed))
      * 12_V));
  } else {
    driveMotor.SetControl(driveVelocitySetter.WithVelocity(velocityToGoTo));
  }
}

void SwerveModule::ResetPosition() { driveMotor.SetPosition(0_rad); }

void SwerveModule::SetSteerVoltage(units::volt_t volts)
{
  steerMotor.SetControl(steerVoltageSetter.WithOutput(volts));
}

void SwerveModule::SetDriveVoltage(units::volt_t volts)
{
  driveMotor.SetControl(driveVoltageSetter.WithOutput(volts));
}

std::array<ctre::phoenix6::BaseStatusSignal*, 6> SwerveModule::GetSignals()
{
  return {&steerAngleSignal, &steerAngleVelocitySignal, &steerVoltageSignal,
    &drivePositionSignal, &driveVelocitySignal, &driveVoltageSignal};
}

void SwerveModule::OptimizeBusSignals()
{
  auto driveStatus = driveMotor.OptimizeBusUtilization();
  if (!driveStatus.IsOK()) {
    frc::DataLogManager::Log(
      fmt::format("Swerve Drive Motor was unable to optimize its bus signals! "
                  "Error: {}, More Info: {}",
        driveStatus.GetName(), driveStatus.GetDescription()));
  }
  auto steerStatus = steerMotor.OptimizeBusUtilization();
  if (!steerStatus.IsOK()) {
    frc::DataLogManager::Log(
      fmt::format("Swerve Steer Motor was unable to optimize its bus signals! "
                  "Error: {}, More Info: {}",
        steerStatus.GetName(), steerStatus.GetDescription()));
  }
}

ctre::phoenix::StatusCode SwerveModule::ConfigureDriveMotor(
  bool invertDrive, units::scalar_t driveGearing, units::ampere_t slipCurrent)
{
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};

  // Be careful here. Make sure we are controlling in volts as gains will be
  // different depending on control mode.
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};
  driveSlotConfig.kV = currentDrivingGains.kV.value();
  driveSlotConfig.kA = currentDrivingGains.kA.value();
  driveSlotConfig.kS = currentDrivingGains.kS.value();
  driveSlotConfig.kP = currentDrivingGains.kP;
  driveSlotConfig.kI = currentDrivingGains.kI;
  driveSlotConfig.kD = currentDrivingGains.kD;
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode
    = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Feedback.SensorToMechanismRatio = driveGearing;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = slipCurrent.value();
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -slipCurrent.value();
  driveConfig.CurrentLimits.StatorCurrentLimit = slipCurrent.value();
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.MotorOutput.Inverted = invertDrive
    ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
    : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  return driveMotor.GetConfigurator().Apply(driveConfig);
}

ctre::phoenix::StatusCode SwerveModule::ConfigureSteerMotor(bool invertSteer,
  units::scalar_t steerGearing,
  const units::radians_per_second_t steerMotionMagicVel,
  const units::radians_per_second_squared_t steerMotionMagicAccel)
{
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};

  // Be careful here. Make sure we are controlling in volts as gains will be
  // different depending on control mode.
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV = currentSteeringGains.kV.value();
  steerSlotConfig.kA = currentSteeringGains.kA.value();
  steerSlotConfig.kS = currentSteeringGains.kS.value();
  steerSlotConfig.kP = currentSteeringGains.kP;
  steerSlotConfig.kI = currentSteeringGains.kI;
  steerSlotConfig.kD = currentSteeringGains.kD;
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotorOutput.NeutralMode
    = ctre::phoenix6::signals::NeutralModeValue::Brake;
  // TODO: Not sure if this is correct...
  steerConfig.Feedback.SensorToMechanismRatio = steerGearing;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource
    = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio = steerGearing;
  steerConfig.MotorOutput.Inverted = invertSteer
    ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
    : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  steerConfig.MotionMagic.MotionMagicAcceleration
    = steerMotionMagicAccel.value();
  steerConfig.MotionMagic.MotionMagicCruiseVelocity
    = steerMotionMagicVel.value();

  return steerMotor.GetConfigurator().Apply(steerConfig);
}

ctre::phoenix::StatusCode SwerveModule::ConfigureSteerEncoder(
  double encoderOffset)
{
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
  return steerEncoder.GetConfigurator().Apply(encoderConfig);
}

units::meter_t SwerveModule::ConvertOutputShaftToWheelDistance(
  units::radian_t shaftRotations) const
{
  return units::ConvertAngularDistanceToLinearDistance(
    shaftRotations, wheelRadius);
}

units::meters_per_second_t SwerveModule::ConvertOutputShaftToWheelVelocity(
  units::radians_per_second_t shaftVelocity) const
{
  return units::ConvertAngularVelocityToLinearVelocity(
    shaftVelocity, wheelRadius);
}

units::radian_t SwerveModule::ConvertWheelDistanceToMotorShaftRotations(
  units::meter_t wheelRotations) const
{
  return units::ConvertLinearDistanceToAngularDistance(
    wheelRotations, wheelRadius);
}

units::radians_per_second_t SwerveModule::ConvertWheelVelocityToMotorVelocity(
  units::meters_per_second_t wheelVelocity) const
{
  return units::ConvertLinearVelocityToAngularVelocity(
           wheelVelocity, wheelRadius)
    * driveGearing;
}

units::radian_t SwerveModule::ConvertOutputShaftPositionToMotorShaftPosition(
  units::radian_t outputShaftPosition) const
{
  return outputShaftPosition * steerGearing;
}

units::radians_per_second_t
SwerveModule::ConvertOutputShaftVelocityToMotorShaftVelocity(
  units::radians_per_second_t outputShaftVelocity) const
{
  return outputShaftVelocity * steerGearing;
}
