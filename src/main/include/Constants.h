// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/velocity.h>

#include "frc/system/plant/DCMotor.h"
#include "str/Units.h"

namespace constants {
namespace swerve {
  namespace can {

  } // namespace can

  namespace physical {
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 4_in;
    static constexpr units::scalar_t DRIVE_GEARING
      = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS L2
    static constexpr units::meters_per_second_t MAX_LINEAR_SPEED
      = units::ConvertAngularVelocityToLinearVelocity(
        frc::DCMotor::Falcon500(1).freeSpeed / DRIVE_GEARING,
        DRIVE_WHEEL_DIAMETER / 2);
    static constexpr units::scalar_t STEER_GEARING
      = (50.0 / 14.0) * (10.0 / 60.0);
  } // namespace physical

} // namespace swerve
} // namespace constants
