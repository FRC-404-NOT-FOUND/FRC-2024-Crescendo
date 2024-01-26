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

package com.argsrobotics.crescendo2024.subsystems.drive;

import static com.argsrobotics.crescendo2024.Constants.Drive.kDriveGearRatio;
import static com.argsrobotics.crescendo2024.Constants.Drive.kTurnGearRatio;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two custom flywheel sims for the drive and turn motors, with the absolute position
 * initialized to a random value. The flywheel sims are not physically accurate, but provide a
 * decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private DCMotor driveSim = DCMotor.getNEO(1);
  private DCMotor turnSim = DCMotor.getNeo550(1);

  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private double turnVelocity = 0.0;
  private double turnPosition = 0.0;
  private double driveVelocity = 0.0;
  private double drivePosition = 0.0;
  private double driveCurrent = 0.0;
  private double turnCurrent = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // I didn't like the default implementation
    // So I did it myself with some oversimplified calculus
    driveVelocity =
        (driveAppliedVolts / 12.0)
            * Units.rotationsPerMinuteToRadiansPerSecond(5676)
            * (1.0 / kDriveGearRatio);
    drivePosition += driveVelocity * LOOP_PERIOD_SECS;
    turnVelocity =
        (turnAppliedVolts / 12.0)
            * Units.rotationsPerMinuteToRadiansPerSecond(11000)
            * (1.0 / kTurnGearRatio);
    turnPosition += turnVelocity * LOOP_PERIOD_SECS;
    driveCurrent = getAppliedCurrent(driveSim, driveVelocity, driveAppliedVolts);
    turnCurrent = getAppliedCurrent(turnSim, turnVelocity, turnAppliedVolts);

    inputs.drivePositionRad = drivePosition;
    inputs.driveVelocityRadPerSec = driveVelocity;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveCurrent)};

    inputs.turnAbsolutePosition = new Rotation2d(turnPosition).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnPosition);
    inputs.turnVelocityRadPerSec = turnVelocity;
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnCurrent)};

    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  private double getAppliedCurrent(DCMotor motor, double velocity, double volts) {
    return motor.getCurrent(velocity, volts) * Math.signum(volts);
  }
}
