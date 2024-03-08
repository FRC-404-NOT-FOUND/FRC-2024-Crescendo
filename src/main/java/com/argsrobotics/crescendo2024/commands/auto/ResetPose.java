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

package com.argsrobotics.crescendo2024.commands.auto;

import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ResetPose extends Command {
  private Drive drive;
  private Position position;

  public static enum Position {
    LEFT(new Pose2d(0.77, 6.8, Rotation2d.fromDegrees(58))),
    CENTER(new Pose2d(1.49, 5.54, new Rotation2d(0.0))),
    RIGHT(new Pose2d(0.79, 4.30, Rotation2d.fromDegrees(-58)));

    private final Pose2d position;

    Position(Pose2d pose) {
      this.position = pose;
    }

    public Pose2d getPose() {
      return position;
    }
  }

  public ResetPose(Drive drive, Position position) {
    this.drive = drive;
    this.position = position;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.resetOdometry(position.getPose());
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
