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

package com.argsrobotics.crescendo2024.subsystems.arm;

import static com.argsrobotics.crescendo2024.Constants.Arm.kGearRatio;
import static com.argsrobotics.crescendo2024.Constants.Arm.kZeroAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The Arm subsystem. */
public class Arm extends SubsystemBase implements AutoCloseable {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (RobotState.isDisabled()) {
      io.setPercent(0);
      io.setPosition(null);
    }

    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/CurrentAngleDegrees", getAngle().getDegrees());
  }

  /**
   * Sets the angle of the arm.
   *
   * @param angle the new angle to set
   */
  public void setAngle(Rotation2d angle) {
    if (angle.equals(kZeroAngle)) {
      io.setPosition(
          (angle.minus(kZeroAngle).plus(Rotation2d.fromDegrees(2))).getRotations() * kGearRatio);
    } else {
      io.setPosition((angle.minus(kZeroAngle)).getRotations() * kGearRatio);
    }
  }

  /**
   * Set the percent output of the arm.
   *
   * @return
   */
  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  /**
   * Set the arm speed using the specified speed supplier. If the speed is zero, the arm is set to
   * the current angle so that PID and FF keep the arm stable.
   *
   * @param speed the supplier for the arm speed
   * @return the command for setting the arm speed
   */
  public Command setArmSpeed(DoubleSupplier speed) {
    return runEnd(() -> setPercent(speed.getAsDouble()), () -> setPercent(0));
  }

  /**
   * Sets the arm angle to the specified value.
   *
   * @param angle the angle to set the arm to
   * @return a Command object representing the set arm angle command
   */
  public Command setArmAngle(Rotation2d angle) {
    return run(() -> setAngle(angle));
  }

  /**
   * Get the angle of the arm.
   *
   * @return the resulting angle
   */
  @AutoLogOutput(key = "Arm/CurrentAngle")
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(inputs.position / kGearRatio).plus(kZeroAngle);
  }

  @Override
  public void close() {
    io.close();
  }
}
