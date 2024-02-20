// FRC 404's 2024 Robot code.
// Copyright (C) 2024 FRC 404

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

package com.argsrobotics.crescendo2024.commands;

import static com.argsrobotics.crescendo2024.Constants.Drive.kDriveDeadband;

import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier corXSupplier,
      DoubleSupplier corYSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), kDriveDeadband);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kDriveDeadband);

          double corX = MathUtil.applyDeadband(corXSupplier.getAsDouble(), kDriveDeadband);
          double corY = MathUtil.applyDeadband(corYSupplier.getAsDouble(), kDriveDeadband);
          Translation2d centerOfRot = new Translation2d(corX, corY);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Convert to chassis speeds and apply slew rate
          ChassisSpeeds speeds =
              drive.calculateSlewRate(linearDirection.getRadians(), linearMagnitude, omega);

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()), centerOfRot);
        },
        drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return joystickDrive(drive, xSupplier, ySupplier, omegaSupplier, () -> 0.0, () -> 0.0);
  }
}
