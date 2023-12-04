// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DrivebaseSubsystem.h"

DrivebaseSubsystem::DrivebaseSubsystem() = default;

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic() { swerveDrive.Log(); }

void DrivebaseSubsystem::UpdateOdometry() { swerveDrive.UpdateOdometry(); }
