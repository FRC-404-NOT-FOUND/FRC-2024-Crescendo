// FRC 404's 2024 Robot code.
// fortnite monkey balls
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

import static com.argsrobotics.crescendo2024.Constants.Arm.kDownAngle;
import static com.argsrobotics.crescendo2024.Constants.Arm.kZeroAngle;

import com.argsrobotics.lib.util.CountingDelay;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** The Arm subsystem. */
public class Arm extends SubsystemBase implements AutoCloseable {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private static final InterpolatingDoubleTreeMap aimer = new InterpolatingDoubleTreeMap();

  private final CountingDelay downDelay = new CountingDelay();

  private Rotation2d positionSetpoint = kZeroAngle;
  private Rotation2d finalPositionSetpoint = kZeroAngle;

  public Arm(ArmIO io) {
    this.io = io;

    aimer.put(0.9607 - 0.2617, kDownAngle.getRadians());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (RobotState.isDisabled()) {
      io.setPercent(0);
      io.setPosition(null);
    }

    if (!finalPositionSetpoint.equals(positionSetpoint) && downDelay.delay(1)) {
      positionSetpoint = positionSetpoint.minus(Rotation2d.fromDegrees(15));
      if (positionSetpoint.getDegrees() < finalPositionSetpoint.getDegrees()) {
        positionSetpoint = finalPositionSetpoint;
      }

      downDelay.reset();
    }

    if (kDownAngle.getDegrees() > positionSetpoint.getDegrees()) {
      io.setPosition(kDownAngle.getRotations());
    } else if (95 < positionSetpoint.getDegrees()) {
      io.setPosition(Units.degreesToRotations(95));
    } else {
      io.setPosition(positionSetpoint.getRotations());
    }

    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/CurrentAngleDegrees", getAngle().getDegrees());
    com.argsrobotics.crescendo2024.RobotState.getCurrentRobotState().armAngle = getAngle();
  }

  /**
   * Sets the angle of the arm.
   *
   * @param angle the new angle to set
   */
  public void setAngle(Rotation2d angle) {
    if (angle.getRotations() > inputs.position) {
      finalPositionSetpoint = angle;
      if (finalPositionSetpoint.getDegrees() >= 95) {
        finalPositionSetpoint = Rotation2d.fromDegrees(95);
      } else if (finalPositionSetpoint.getDegrees() <= kDownAngle.getDegrees()) {
        finalPositionSetpoint = kDownAngle;
      }
      positionSetpoint = angle;
    } else {
      finalPositionSetpoint = angle;
      if (finalPositionSetpoint.getDegrees() >= 95) {
        finalPositionSetpoint = Rotation2d.fromDegrees(95);
      } else if (finalPositionSetpoint.getDegrees() <= kDownAngle.getDegrees()) {
        finalPositionSetpoint = kDownAngle;
      }
      positionSetpoint =
          Rotation2d.fromRotations(inputs.position).minus(Rotation2d.fromDegrees(15));
      downDelay.reset();
    }
  }

  /**
   * Sets the angle of the arm.
   *
   * @param angle the new angle to set
   */
  public void setAngle(ArmAngle angle) {
    Rotation2d targetAngle =
        angle.getAngle(
            com.argsrobotics.crescendo2024.RobotState.getCurrentRobotState().currentPose);
    setAngle(targetAngle);
  }

  /**
   * Set the percent output of the arm.
   *
   * @return
   */
  public void setPercent(double percent) {
    io.setPercent(percent * 0.5);
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
    return new SequentialCommandGroup(
        runOnce(() -> setAngle(angle)).ignoringDisable(true), Commands.waitSeconds(7));
  }

  /**
   * Sets the arm angle to the specified value.
   *
   * @param angle the angle to set the arm to
   * @return a Command object representing the set arm angle command
   */
  public Command setArmAngle(ArmAngle angle) {
    return run(() -> setAngle(angle));
  }

  /** Get the arm free at the beginning of the match */
  public Command cutLoose() {
    return Commands.sequence(
        setArmAngle(Rotation2d.fromDegrees(95)).withTimeout(2), setArmAngle(kDownAngle));
  }

  /** Climb down */
  public Command climbDown() {
    return run(() -> io.setPosition(Rotation2d.fromDegrees(0).getRotations()))
        .ignoringDisable(true);
  }

  public Command holdArm() {
    return setArmAngle(kZeroAngle);
  }

  /**
   * Get the angle of the arm.
   *
   * @return the resulting angle
   */
  @AutoLogOutput(key = "Arm/CurrentAngle")
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(inputs.position);
  }

  @Override
  public void close() {
    io.close();
  }

  public static enum ArmAngle {
    SUBWOOFER(kDownAngle),
    PODIUM(kDownAngle),
    INTAKE(kDownAngle),
    AMP(Rotation2d.fromDegrees(90)),
    AUTO(new Rotation2d());

    private Rotation2d angle;

    private ArmAngle(Rotation2d angle) {
      this.angle = angle;
    }

    public Rotation2d getAngle(Pose2d currentPose) {
      if (this == ArmAngle.AUTO) {
        Pose2d speakerPose;
        if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red) {
          speakerPose = new Pose2d(16.3, 5.549, new Rotation2d());
        } else {
          speakerPose = new Pose2d(0.2167, 5.549, new Rotation2d());
        }

        double distance =
            Math.abs(currentPose.getTranslation().getDistance(speakerPose.getTranslation()));

        return new Rotation2d(aimer.get(distance));
      }

      return this.angle;
    }
  }
}
