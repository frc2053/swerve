// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "choreo/lib/ChoreoSwerveCommand.h"

#include <frc/DriverStation.h>

using namespace choreolib;

ChoreoSwerveCommand::ChoreoSwerveCommand(ChoreoTrajectory trajectory,
  std::function<frc::Pose2d()> poseSupplier,
  ChoreoControllerFunction controller,
  std::function<void(frc::ChassisSpeeds)> outputChassisSpeeds,
  bool useAllianceColor, frc2::Requirements requirements)
  : m_traj(trajectory)
  , m_pose(poseSupplier)
  , m_controller(controller)
  , m_outputChassisSpeeds(outputChassisSpeeds)
  , m_useAlliance(useAllianceColor)
{
  AddRequirements(requirements);
}

// Called when the command is initially scheduled.
void ChoreoSwerveCommand::Initialize() { m_timer.Restart(); }

// Called repeatedly when this Command is scheduled to run
void ChoreoSwerveCommand::Execute()
{
  bool mirror = false;
  if (m_useAlliance) {
    std::optional<frc::DriverStation::Alliance> alliance
      = frc::DriverStation::GetAlliance();
    mirror = alliance.has_value()
      && alliance.value() == frc::DriverStation::Alliance::kRed;
  }
  units::second_t currentTrajTime = m_timer.Get();
  m_outputChassisSpeeds(
    m_controller(m_pose(), m_traj.Sample(currentTrajTime, mirror)));
}

// Called once the command ends or is interrupted.
void ChoreoSwerveCommand::End(bool interrupted)
{
  m_outputChassisSpeeds(frc::ChassisSpeeds{});
}

// Returns true when the command should end.
bool ChoreoSwerveCommand::IsFinished()
{
  bool isDone = m_timer.HasElapsed(m_traj.GetTotalTime());
  return isDone;
}
