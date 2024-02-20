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

import static com.argsrobotics.crescendo2024.Constants.Arm.kZeroAngle;

import com.argsrobotics.crescendo2024.subsystems.arm.Arm;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TuneArmPID extends Command {
  private final Arm arm;

  public TuneArmPID(Arm a) {
    this.arm = a;
    SmartDashboard.putNumber("Arm/SetpointTuning", kZeroAngle.getDegrees());
    addRequirements(this.arm);
  }

  @Override
  public void execute() {
    arm.setAngle(
        Rotation2d.fromDegrees(
            SmartDashboard.getNumber("Arm/SetpointTuning", kZeroAngle.getDegrees())));
  }

  @Override
  public void end(boolean interruped) {
    arm.setAngle(kZeroAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
