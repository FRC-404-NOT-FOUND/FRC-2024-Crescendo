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

import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneDrivePID extends Command {
  private final Drive drive;

  public TuneDrivePID(Drive d) {
    this.drive = d;
    SmartDashboard.putNumber("Drive/VelocitySetpointTuning", 0);
    SmartDashboard.putNumber("Drive/AngleSetpointTuning", 0);
    addRequirements(this.drive);
  }

  @Override
  public void execute() {
    drive.runSetpoint(
        SmartDashboard.getNumber("Drive/VelocitySetpointTuning", 0),
        Rotation2d.fromDegrees(SmartDashboard.getNumber("Drive/AngleSetpointTuning", 0)));
  }

  @Override
  public void end(boolean interruped) {
    drive.runSetpoint(0, Rotation2d.fromDegrees(0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
