// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDrive.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "frc/RobotBase.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Transform2d.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/smartdashboard/FieldObject2d.h"

using namespace str;

SwerveDrive::SwerveDrive()
{
  frc::SmartDashboard::PutData(&ntField);

  for (int i = 0; i < 4; i++) {
    const auto& moduleSignals = swerveModules[i].GetSignals();
    allModuleSignals[(i * 4) + 0] = moduleSignals[0]; // steer pos
    allModuleSignals[(i * 4) + 1] = moduleSignals[1]; // steer vel
    allModuleSignals[(i * 4) + 2] = moduleSignals[3]; // drive pos
    allModuleSignals[(i * 4) + 3] = moduleSignals[4]; // drive vel
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

void SwerveDrive::Log()
{
  frc::SmartDashboard::PutNumber(
    "Drivebase/Averge Odom Frequency", (1 / averageLoopTime).value());

  ntField.GetObject("Estimated Robot Pose")->SetPose(GetPose());
  ntField.GetObject("Estimated Robot Modules")->SetPoses(GetModulePoses());
}

frc::Pose2d SwerveDrive::GetPose() const
{
  return poseEstimator.GetEstimatedPosition();
}

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

  units::radian_t imuYaw
    = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      imu.GetYaw(), imu.GetAngularVelocityZ());
  poseEstimator.Update(frc::Rotation2d{imuYaw}, modulePostions);
}
