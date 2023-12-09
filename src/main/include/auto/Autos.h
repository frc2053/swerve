// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "subsystems/DrivebaseSubsystem.h"

namespace autos {

enum CommandSelector { DO_NOTHING, THREE_FT_FORWARD, SQUARE };

class Autos {
public:
  explicit Autos(DrivebaseSubsystem& driveSub)
    : m_driveSub(driveSub)
  {
    chooser.SetDefaultOption("Do Nothing", CommandSelector::DO_NOTHING);
    chooser.AddOption("Three Feet Forward", CommandSelector::THREE_FT_FORWARD);
    chooser.AddOption("Drive in Square", CommandSelector::SQUARE);

    frc::SmartDashboard::PutData("Auto Chooser", &chooser);
  }

  DrivebaseSubsystem& m_driveSub;
  frc::SendableChooser<CommandSelector> chooser;

  frc2::CommandPtr GetSelectedAutoCmd = frc2::cmd::Select<CommandSelector>(
    [this] { return chooser.GetSelected(); },
    std::pair{DO_NOTHING,
      frc2::cmd::Print(
        "ERROR: DO NOTHING AUTO SELECTED! YOU PROBABLY DIDNT MEAN THIS")},
    std::pair{THREE_FT_FORWARD,
      m_driveSub.FollowChoreoTrajectory([] { return "ThreeFeetForward"; })},
    std::pair{
      SQUARE, m_driveSub.FollowChoreoTrajectory([] { return "Square"; })});
};
} // namespace autos
