// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/MathUtil.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

namespace units {
constexpr units::meter_t ConvertEncoderTicksToDistance(
  int ticks, int encoderResolution, double gearing, units::meter_t wheelRadius)
{
  return (ticks / (encoderResolution * gearing))
    * (2 * std::numbers::pi * wheelRadius);
}

constexpr units::meter_t ConvertAngularDistanceToLinearDistance(
  units::radian_t turns, units::meter_t wheelRadius)
{
  // https://www.calculussolution.com/blog/arc-length
  // radians are actually dimensionless
  return (turns / 1_rad) * wheelRadius;
}

constexpr units::radian_t ConvertLinearDistanceToAngularDistance(
  units::meter_t distance, units::meter_t wheelRadius)
{
  return (distance / wheelRadius) * 1_rad;
}

constexpr units::radian_t ConvertTicksToAngle(
  int ticks, int encoderResolution, double gearing, bool wrap = true)
{
  auto angle = (ticks / (encoderResolution * gearing))
    * units::radian_t{2 * std::numbers::pi};
  return wrap ? frc::AngleModulus(angle) : angle;
}

constexpr int ConvertDistanceToEncoderTicks(units::meter_t distance,
  int encoderResolution, double gearing, units::meter_t wheelRadius)
{
  return distance * (encoderResolution * gearing)
    / (std::numbers::pi * 2 * wheelRadius);
}

constexpr int ConvertAngleToEncoderTicks(units::radian_t angle,
  int encoderResolution, double gearing, bool wrap = true)
{
  if (wrap) {
    angle = frc::AngleModulus(angle);
  }
  return angle * (encoderResolution * gearing)
    / units::radian_t{std::numbers::pi * 2};
}

constexpr units::radians_per_second_t ConvertLinearVelocityToAngularVelocity(
  units::meters_per_second_t linearVelocity, units::meter_t radius)
{
  // https://www.calculussolution.com/blog/arc-length
  // radians are actually dimensionless
  return linearVelocity / (radius / 1_rad);
}

constexpr units::meters_per_second_t ConvertAngularVelocityToLinearVelocity(
  units::radians_per_second_t angularVelocity, units::meter_t radius)
{
  // https://www.calculussolution.com/blog/arc-length
  // radians are actually dimensionless
  return angularVelocity * (radius / 1_rad);
}

template <typename T>
  requires std::is_arithmetic_v<T> || units::traits::is_unit_t_v<T>
constexpr T Map(T input, T minIn, T maxIn, T minOut, T maxOut)
{
  return (input - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
}

template <typename T> constexpr int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

} // namespace units
