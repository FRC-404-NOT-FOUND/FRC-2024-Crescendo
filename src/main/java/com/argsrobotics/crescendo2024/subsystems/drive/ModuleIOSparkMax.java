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

import static com.argsrobotics.crescendo2024.Constants.Drive.*;
import static com.argsrobotics.crescendo2024.Constants.kTuningMode;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.OptionalDouble;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * 550), and Rev through bore absolute encoders based off of AdvantageKit template
 * (https://github.com/Mechanical-Advantage/AdvantageKit).
 */
public class ModuleIOSparkMax implements ModuleIO {
  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private SimpleMotorFeedforward driveFeedforward;
  private final SparkPIDController drivePidController;
  private final SparkPIDController turnPidController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> timestampQueue;

  private final boolean isTurnEncoderInverted = true;
  private double chassisAngularOffset = 0.0;

  public ModuleIOSparkMax(int index, double chassisAngularOffset) {
    this.chassisAngularOffset = chassisAngularOffset;
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        break;
      case 1:
        driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        break;
      case 2:
        driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        break;
      case 3:
        driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    // Restore factory defaults
    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    // Set CAN timeouts
    driveSparkMax.setCANTimeout(5);
    turnSparkMax.setCANTimeout(5);

    // Configure encoders and PID controllers
    driveEncoder = driveSparkMax.getEncoder();
    turnEncoder = turnSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivePidController = driveSparkMax.getPIDController();
    turnPidController = turnSparkMax.getPIDController();

    // Set current limits and enable voltage compensation
    driveSparkMax.setSmartCurrentLimit(50);
    turnSparkMax.setSmartCurrentLimit(20);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    // Encoder settings
    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnEncoder.setAverageDepth(2);
    turnEncoder.setInverted(isTurnEncoderInverted);

    // Configure turn PID controller for wrapped position
    turnPidController.setPositionPIDWrappingEnabled(true);
    turnPidController.setPositionPIDWrappingMinInput(-Math.PI);
    turnPidController.setPositionPIDWrappingMaxInput(Math.PI);

    // Set position and velocity conversion factors
    driveEncoder.setPositionConversionFactor((kWheelRadius * 2 * Math.PI) / kDriveGearRatio);
    driveEncoder.setVelocityConversionFactor(
        ((2 * Math.PI * kWheelRadius) / kDriveGearRatio) / 60.0);
    turnEncoder.setPositionConversionFactor(2 * Math.PI);
    turnEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

    // Set feedback devices for PID controllers
    drivePidController.setFeedbackDevice(driveEncoder);
    turnPidController.setFeedbackDevice(turnEncoder);

    // Create and configure drive feedforward
    driveFeedforward = new SimpleMotorFeedforward(kDriveS.get(), kDriveV.get());
    drivePidController.setP(kDriveP.get());
    drivePidController.setI(kDriveI.get());
    drivePidController.setD(kDriveD.get());
    drivePidController.setFF(0);
    drivePidController.setOutputRange(-1, 1);

    // Configure turn PID controller
    turnPidController.setP(kTurnP.get());
    turnPidController.setI(kTurnI.get());
    turnPidController.setD(kTurnD.get());
    turnPidController.setFF(kTurnFF.get());
    turnPidController.setOutputRange(-1, 1);

    // Set periodic frame periods
    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / kOdometryFrequency));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / kOdometryFrequency));

    // Register signals for position and timestamp queues
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  return (driveSparkMax.getLastError() == REVLibError.kOk)
                      ? OptionalDouble.of(driveEncoder.getPosition())
                      : OptionalDouble.empty();
                });

    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  return (turnSparkMax.getLastError() == REVLibError.kOk)
                      ? OptionalDouble.of(turnEncoder.getPosition())
                      : OptionalDouble.empty();
                });

    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();

    // Burn flash for Spark Max controllers
    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (kTuningMode) {
      if (kDriveP.get() != drivePidController.getP()) {
        drivePidController.setP(kDriveP.get());
      }

      if (kDriveI.get() != drivePidController.getI()) {
        drivePidController.setI(kDriveI.get());
      }

      if (kDriveD.get() != drivePidController.getD()) {
        drivePidController.setD(kDriveD.get());
      }

      if (kTurnP.get() != turnPidController.getP()) {
        turnPidController.setP(kTurnP.get());
      }

      if (kTurnI.get() != turnPidController.getI()) {
        turnPidController.setI(kTurnI.get());
      }

      if (kTurnD.get() != turnPidController.getD()) {
        turnPidController.setD(kTurnD.get());
      }
      driveFeedforward = new SimpleMotorFeedforward(kDriveS.get(), kDriveV.get());
    }

    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    double position = turnEncoder.getPosition() - chassisAngularOffset;

    // Stupid encoders
    // if (SwerveUtils.angleDifference(position, inputs.turnPosition.getRadians()) <= 0.01) {
    //   position = inputs.turnPosition.getRadians();
    // }

    inputs.turnPosition = Rotation2d.fromRadians(position);
    inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream().mapToDouble((v) -> v).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);

    drivePositionQueue.clear();
    turnPositionQueue.clear();
    timestampQueue.clear();
  }

  @Override
  public double getAngularOffset() {
    return chassisAngularOffset;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  public void setDriveVelocity(double velocity) {
    drivePidController.setReference(
        velocity, ControlType.kVelocity, 0, driveFeedforward.calculate(velocity));
  }

  public void setTurnAngle(Rotation2d angle) {
    turnPidController.setReference(
        angle.getRadians() + chassisAngularOffset, ControlType.kPosition);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void close() {
    driveSparkMax.close();
    turnSparkMax.close();
  }
}
