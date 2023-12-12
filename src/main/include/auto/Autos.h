// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/PrintCommand.h"
#include "subsystems/DrivebaseSubsystem.h"

namespace autos {

enum CommandSelector {
  DO_NOTHING,
  THREE_FT_FORWARD,
  SQUARE,
  TEST,
  PATHPLANNER_TEST
};

class Autos {
public:
  explicit Autos(DrivebaseSubsystem& driveSub)
    : m_driveSub(driveSub)
  {
    pathplanner::NamedCommands::registerCommand("TestCommandPrint",
      frc2::PrintCommand("Test Print from PP Command").ToPtr());

    GetSelectedAutoCmd = frc2::cmd::Select<CommandSelector>(
      [this] { return chooser.GetSelected(); },
      std::pair{DO_NOTHING,
        frc2::cmd::Print(
          "ERROR: DO NOTHING AUTO SELECTED! YOU PROBABLY DIDNT MEAN THIS")},
      std::pair{THREE_FT_FORWARD,
        m_driveSub.FollowChoreoTrajectory([] { return "ThreeFeetForward"; })},
      std::pair{
        SQUARE, m_driveSub.FollowChoreoTrajectory([] { return "Square"; })},
      std::pair{TEST, m_driveSub.FollowChoreoTrajectory([] { return "Test"; })},
      std::pair{
        PATHPLANNER_TEST, pathplanner::PathPlannerAuto{"PPTest"}.ToPtr()});

    chooser.SetDefaultOption("Do Nothing", CommandSelector::DO_NOTHING);
    chooser.AddOption("Three Feet Forward", CommandSelector::THREE_FT_FORWARD);
    chooser.AddOption("Drive in Square", CommandSelector::SQUARE);
    chooser.AddOption("Test Trajectory", CommandSelector::TEST);
    chooser.AddOption("Pathplanner Test", CommandSelector::PATHPLANNER_TEST);

    frc::SmartDashboard::PutData("Auto Chooser", &chooser);
  }

  DrivebaseSubsystem& m_driveSub;
  frc::SendableChooser<CommandSelector> chooser;

  frc2::CommandPtr GetSelectedAutoCmd{frc2::cmd::None()};
};
} // namespace autos
