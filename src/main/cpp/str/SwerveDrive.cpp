// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDrive.h"

#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/TimedRobot.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/current.h>

#include <iostream>

#include "Constants.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "str/SwerveDriveSim.h"

using namespace str;

SwerveDrive::SwerveDrive()
{
  frc::SmartDashboard::PutData(&ntField);

  for (int i = 0; i < 4; i++) {
    const auto& moduleSignals = swerveModules[i].GetSignals();
    allModuleSignals[(i * 6) + 0] = moduleSignals[0]; // steer pos
    allModuleSignals[(i * 6) + 1] = moduleSignals[1]; // steer vel
    allModuleSignals[(i * 6) + 2] = moduleSignals[2]; // steer vol
    allModuleSignals[(i * 6) + 3] = moduleSignals[3]; // drive pos
    allModuleSignals[(i * 6) + 4] = moduleSignals[4]; // drive vel
    allModuleSignals[(i * 6) + 5] = moduleSignals[5]; // drive vol
  }
  allModuleSignals[allModuleSignals.size() - 2] = &imu.GetYaw();
  allModuleSignals[allModuleSignals.size() - 1] = &imu.GetAngularVelocityZ();

  for (const auto& signal : allModuleSignals) {
    ctre::phoenix::StatusCode status = signal->SetUpdateFrequency(250_Hz);
    if (!status.IsOK()) {
      frc::DataLogManager::Log(
        fmt::format("Signal {} was unable to set its frequence to 250 Hz! "
                    "Error: {}, More Info: {}",
          signal->GetName(), status.GetName(), status.GetDescription()));
    }
  }

  imu.OptimizeBusUtilization();
  for (int i = 0; i < 4; i++) {
    swerveModules[i].OptimizeBusSignals();
  }
}

void SwerveDrive::Drive(units::meters_per_second_t vx,
  units::meters_per_second_t vy, units::radians_per_second_t omega,
  bool openLoop)
{
  frc::ChassisSpeeds newChassisSpeeds
    = frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, GetHeading());
  SetChassisSpeeds(newChassisSpeeds, openLoop);
}

void SwerveDrive::SetChassisSpeeds(
  const frc::ChassisSpeeds& newChassisSpeeds, bool openLoop)
{
  SetModuleStates(constants::swerve::physical::KINEMATICS.ToSwerveModuleStates(
                    newChassisSpeeds),
    openLoop);
}

void SwerveDrive::SetModuleStates(
  const std::array<frc::SwerveModuleState, 4>& desiredStates, bool openLoop)
{
  units::meters_per_second_t maxSpeed;
  if (openLoop) {
    maxSpeed = constants::swerve::physical::MAX_LINEAR_SPEED;
  } else {
    maxSpeed = constants::swerve::physical::MAX_LINEAR_SPEED_FOC;
  }

  std::array<frc::SwerveModuleState, 4> desaturatedStates = desiredStates;
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
    static_cast<wpi::array<frc::SwerveModuleState, 4>*>(&desaturatedStates),
    maxSpeed);
  for (int i = 0; i < swerveModules.size(); i++) {
    swerveModules[i].GoToState(desaturatedStates[i], openLoop);
  }
}

frc::Rotation2d SwerveDrive::GetHeading() const { return GetPose().Rotation(); }

frc::Rotation2d SwerveDrive::GetGyroYaw() const
{
  return frc::Rotation2d{imuYaw};
}

void SwerveDrive::Log()
{
  frc::SmartDashboard::PutNumber(
    "Drivebase/Averge Odom Frequency", (1 / averageLoopTime).value());

  ntField.GetObject("Estimated Robot Pose")->SetPose(GetPose());
  ntField.GetObject("Estimated Robot Modules")->SetPoses(GetModulePoses());

  for (int i = 0; i < swerveModules.size(); i++) {
    swerveModules[i].Log(i);
  }
}

void SwerveDrive::SimulationUpdate()
{
  std::array<units::volt_t, 4> driveInputs;
  std::array<units::volt_t, 4> steerInputs;
  for (int i = 0; i < 4; i++) {
    steerInputs[i]
      = units::volt_t{allModuleSignals[(i * 6) + 2]->GetValueAsDouble()};
    driveInputs[i]
      = units::volt_t{allModuleSignals[(i * 6) + 5]->GetValueAsDouble()};
  }

  swerveSim.SetDriveInputs(driveInputs);
  swerveSim.SetSteerInputs(steerInputs);

  swerveSim.Update(frc::TimedRobot::kDefaultPeriod);

  totalCurrentDraw
    = swerveSim.GetSteerCurrentDraw() + swerveSim.GetDriveCurrentDraw();

  std::array<SimState, 4> state = swerveSim.GetState();
  std::array<frc::SwerveModulePosition, 4> positions{};
  for (int i = 0; i < swerveModules.size(); i++) {
    swerveModules[i].SimulationUpdate(state[i].drivePos, state[i].driveVel,
      state[i].steerPos, state[i].steerVel);
    frc::SwerveModulePosition currentPos = swerveModules[i].GetCachedPosition();
    positions[i] = frc::SwerveModulePosition{
      currentPos.distance - lastPositions[i].distance, currentPos.angle};
    lastPositions[i].distance = currentPos.distance;
  }

  frc::Twist2d change
    = constants::swerve::physical::KINEMATICS.ToTwist2d(positions);
  lastAngle = lastAngle + frc::Rotation2d{change.dtheta};
  imuSimState.SetRawYaw(lastAngle.Degrees());
}

frc::Pose2d SwerveDrive::GetPose() const
{
  return poseEstimator.GetEstimatedPosition();
}

units::ampere_t SwerveDrive::GetCurrentDraw() const { return totalCurrentDraw; }

std::array<frc::Pose2d, 4> SwerveDrive::GetModulePoses() const
{
  std::array<frc::Pose2d, 4> poses;
  for (int i = 0; i < 4; i++) {
    poses[i] = GetPose().TransformBy(
      frc::Transform2d{constants::swerve::physical::moduleLocations[i],
        swerveModules[i].GetCachedPosition().angle});
  }
  return poses;
}

void SwerveDrive::UpdateOdometry()
{
  lastTime = currentTime;
  currentTime = units::second_t{ctre::phoenix6::GetCurrentTimeSeconds()};
  averageLoopTime = currentTime - lastTime;

  ctre::phoenix::StatusCode status
    = ctre::phoenix6::BaseStatusSignal::WaitForAll(
      2.0 / 250_Hz, allModuleSignals);

  if (frc::RobotBase::IsReal()) {
    if (!status.IsOK()) {
      frc::DataLogManager::Log(
        fmt::format("UpdateOdometry failed: {}. More info: {}",
          status.GetName(), status.GetDescription()));
    }
  }

  for (int i = 0; i < 4; i++) {
    modulePostions[i] = swerveModules[i].GetPosition(false);
  }

  imuYaw = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
    imu.GetYaw(), imu.GetAngularVelocityZ());
  poseEstimator.Update(frc::Rotation2d{imuYaw}, modulePostions);
}
