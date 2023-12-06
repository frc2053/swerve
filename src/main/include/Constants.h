// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>

#include "str/Units.h"

namespace constants {
namespace swerve {
  namespace can {
    static constexpr int FL_DRIVE = 2;
    static constexpr int FL_STEER = 3;
    static constexpr int FL_ENC = 4;

    static constexpr int FR_DRIVE = 5;
    static constexpr int FR_STEER = 6;
    static constexpr int FR_ENC = 7;

    static constexpr int BL_DRIVE = 8;
    static constexpr int BL_STEER = 9;
    static constexpr int BL_ENC = 10;

    static constexpr int BR_DRIVE = 11;
    static constexpr int BR_STEER = 12;
    static constexpr int BR_ENC = 13;

    static constexpr int IMU = 14;
  } // namespace can

  namespace physical {
    static constexpr units::meter_t WHEELBASE_LENGTH = 25_in;
    static constexpr units::meter_t WHEELBASE_WIDTH = 25_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 4_in;
    static constexpr units::scalar_t DRIVE_GEARING
      = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS L2
    static constexpr units::scalar_t STEER_GEARING
      = (50.0 / 14.0) * (10.0 / 60.0);
    static constexpr units::scalar_t DRIVE_STEER_COUPLING = (50.0 / 14.0);
    static constexpr units::meters_per_second_t MAX_LINEAR_SPEED
      = units::ConvertAngularVelocityToLinearVelocity(
        frc::DCMotor::Falcon500(1).freeSpeed / DRIVE_GEARING,
        DRIVE_WHEEL_DIAMETER / 2);

    static constexpr double FL_ENCODER_OFFSET = 0;
    static constexpr double FR_ENCODER_OFFSET = 0;
    static constexpr double BL_ENCODER_OFFSET = 0;
    static constexpr double BR_ENCODER_OFFSET = 0;

    static constexpr units::ampere_t SLIP_CURRENT = 400_A;

    static constexpr units::radians_per_second_t STEER_MOTION_MAGIC_VEL
      = 10.0_rad_per_s;
    static constexpr units::radians_per_second_squared_t
      STEER_MOTION_MAGIC_ACCEL{100};

    static constexpr std::array<frc::Translation2d, 4> moduleLocations{
      frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
      frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
      frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
      frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};
  } // namespace physical
} // namespace swerve
} // namespace constants
