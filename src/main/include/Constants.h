// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>

#include "str/Units.h"

namespace constants {
namespace swerve {

  struct ModuleDriveGains {
    units::ka_unit_t kA{0};
    units::kv_unit_t kV{0};
    units::volt_t kS{0};
    units::scalar_t kP{0};
    units::scalar_t kI{0};
    units::scalar_t kD{0};

    bool operator==(const ModuleDriveGains& rhs) const
    {
      return units::essentiallyEqual(kA, rhs.kA, 1e-6)
        && units::essentiallyEqual(kV, rhs.kV, 1e-6)
        && units::essentiallyEqual(kS, rhs.kS, 1e-6)
        && units::essentiallyEqual(kP, rhs.kP, 1e-6)
        && units::essentiallyEqual(kI, rhs.kI, 1e-6)
        && units::essentiallyEqual(kD, rhs.kD, 1e-6);
    }
    bool operator!=(const ModuleDriveGains& rhs) const
    {
      return !operator==(rhs);
    }
  };

  using radial_ka_unit = units::compound_unit<
    units::compound_unit<units::volts, units::squared<units::seconds>>,
    units::inverse<units::radians>>;
  using radial_ka_unit_t = units::unit_t<radial_ka_unit>;

  struct ModuleSteerGains {
    radial_ka_unit_t kA{0};
    frc::DCMotor::radians_per_second_per_volt_t kV{0};
    units::volt_t kS{0};
    units::scalar_t kP{0};
    units::scalar_t kI{0};
    units::scalar_t kD{0};

    bool operator==(const ModuleSteerGains& rhs) const
    {
      return units::essentiallyEqual(kA, rhs.kA, 1e-6)
        && units::essentiallyEqual(kV, rhs.kV, 1e-6)
        && units::essentiallyEqual(kS, rhs.kS, 1e-6)
        && units::essentiallyEqual(kP, rhs.kP, 1e-6)
        && units::essentiallyEqual(kI, rhs.kI, 1e-6)
        && units::essentiallyEqual(kD, rhs.kD, 1e-6);
    }
    bool operator!=(const ModuleSteerGains& rhs) const
    {
      return !operator==(rhs);
    }
  };

  static constexpr ModuleDriveGains driveGains{
    units::ka_unit_t{0.3}, units::kv_unit_t{2.5}, .25_V, 50, 0.0, 0.0};
  static constexpr ModuleSteerGains steerGains{radial_ka_unit_t{0.039166},
    frc::DCMotor::radians_per_second_per_volt_t{0.037461}, 0.0_V, 100, 0.0,
    0.0};

  namespace pathplanning {
    static constexpr units::scalar_t TRANSLATION_P = 1.0;
    static constexpr units::scalar_t TRANSLATION_I = 0.0;
    static constexpr units::scalar_t TRANSLATION_D = 0.0;

    static constexpr units::scalar_t ROTATION_P = 1.0;
    static constexpr units::scalar_t ROTATION_I = 0.0;
    static constexpr units::scalar_t ROTATION_D = 0.0;
  } // namespace pathplanning

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
    static constexpr frc::DCMotor SWERVE_MOTOR{frc::DCMotor::Falcon500(1)};
    static constexpr frc::DCMotor SWERVE_MOTOR_FOC{
      frc::DCMotor::Falcon500FOC(1)};

    static constexpr units::meter_t WHEELBASE_LENGTH = 23.5_in;
    static constexpr units::meter_t WHEELBASE_WIDTH = 23.5_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 4_in;
    static constexpr units::scalar_t DRIVE_GEARING
      = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS L2
    static constexpr units::scalar_t STEER_GEARING
      = (50.0 / 14.0) * (60.0 / 10.0);
    static constexpr units::scalar_t DRIVE_STEER_COUPLING = (50.0 / 14.0);
    static constexpr units::meters_per_second_t MAX_LINEAR_SPEED
      = units::ConvertAngularVelocityToLinearVelocity(
        SWERVE_MOTOR.freeSpeed / DRIVE_GEARING, DRIVE_WHEEL_DIAMETER / 2);
    static constexpr units::meters_per_second_t MAX_LINEAR_SPEED_FOC
      = units::ConvertAngularVelocityToLinearVelocity(
        SWERVE_MOTOR_FOC.freeSpeed / DRIVE_GEARING, DRIVE_WHEEL_DIAMETER / 2);
    static constexpr units::radians_per_second_t MAX_ROTATION_SPEED
      = 720_deg_per_s;

    static constexpr double FL_ENCODER_OFFSET = -0.468262;
    static constexpr double FR_ENCODER_OFFSET = 0.479492;
    static constexpr double BL_ENCODER_OFFSET = 0.359375;
    static constexpr double BR_ENCODER_OFFSET = 0.131104;

    static constexpr units::ampere_t SLIP_CURRENT = 400_A;

    static constexpr std::array<frc::Translation2d, 4> moduleLocations{
      frc::Translation2d{WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
      frc::Translation2d{WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2},
      frc::Translation2d{-WHEELBASE_LENGTH / 2, WHEELBASE_WIDTH / 2},
      frc::Translation2d{-WHEELBASE_LENGTH / 2, -WHEELBASE_WIDTH / 2}};

    static frc::SwerveDriveKinematics<4> KINEMATICS{moduleLocations[0],
      moduleLocations[1], moduleLocations[2], moduleLocations[3]};
  } // namespace physical
} // namespace swerve
} // namespace constants
