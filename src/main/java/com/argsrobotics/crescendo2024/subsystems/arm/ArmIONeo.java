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

import static com.argsrobotics.crescendo2024.Constants.Arm.*;
import static com.argsrobotics.crescendo2024.Constants.kTuningMode;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;

public class ArmIONeo implements ArmIO {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final SparkPIDController leftPidController;
  private final SparkPIDController rightPidController;
  private ArmFeedforward ff = new ArmFeedforward(0, kArmFF.get(), 0);

  private int climb = 1;

  private Double positionSetpoint = null;

  public ArmIONeo(int left, int right) {
    leftMotor = new CANSparkMax(left, MotorType.kBrushless);
    rightMotor = new CANSparkMax(right, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);

    leftMotor.setSmartCurrentLimit(50);
    rightMotor.setSmartCurrentLimit(50);
    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(1 / kGearRatio);
    leftEncoder.setVelocityConversionFactor(1 / kGearRatio);
    rightEncoder.setPositionConversionFactor(1 / kGearRatio);
    rightEncoder.setVelocityConversionFactor(1 / kGearRatio);

    leftEncoder.setPosition(kZeroAngle.getRotations());
    leftEncoder.setMeasurementPeriod(10);
    leftEncoder.setAverageDepth(2);

    rightEncoder.setPosition(kZeroAngle.getRotations());
    rightEncoder.setMeasurementPeriod(10);
    rightEncoder.setAverageDepth(2);

    leftPidController = leftMotor.getPIDController();
    rightPidController = rightMotor.getPIDController();

    leftPidController.setFeedbackDevice(leftEncoder);
    rightPidController.setFeedbackDevice(rightEncoder);

    leftPidController.setP(kArmP.get());
    leftPidController.setI(kArmI.get());
    leftPidController.setD(kArmD.get());
    leftPidController.setFF(0);

    rightPidController.setP(kArmP.get());
    rightPidController.setI(kArmI.get());
    rightPidController.setD(kArmD.get());
    rightPidController.setFF(0);

    leftMotor.setCANTimeout(0);
    rightMotor.setCANTimeout(0);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRotations(95));
    leftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) kDownAngle.getRotations());
    rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float) Units.degreesToRotations(95));
    rightMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) kDownAngle.getRotations());

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (kTuningMode) {
      leftPidController.setP(kArmP.get());
      leftPidController.setI(kArmI.get());
      leftPidController.setD(kArmD.get());
      rightPidController.setP(kArmP.get());
      rightPidController.setI(kArmI.get());
      rightPidController.setD(kArmD.get());
      ff = new ArmFeedforward(0, kArmFF.get(), 0);
    }

    if (positionSetpoint != null) {
      double voltage;
      if (Math.abs(positionSetpoint - kDownAngle.getRotations()) <= 2) {
        voltage = 0;
      } else {
        voltage = ff.calculate(Units.rotationsToRadians(positionSetpoint), 0);
      }
      leftPidController.setReference(
          positionSetpoint, CANSparkBase.ControlType.kPosition, 0, climb * voltage);
      rightPidController.setReference(
          positionSetpoint, CANSparkBase.ControlType.kPosition, 0, climb * voltage);
    }

    inputs.position = leftEncoder.getPosition();
    inputs.velocity = leftEncoder.getVelocity();
    inputs.current = leftMotor.getOutputCurrent();
    inputs.voltage = leftMotor.getBusVoltage() * leftMotor.getAppliedOutput();
  }

  @Override
  public void setPosition(Double position) {
    positionSetpoint = position;
    climb = 1;
  }

  @Override
  public void setClimbAngle(Double angle) {
    positionSetpoint = angle;
    climb = -1;
  }

  @Override
  public void setPercent(double percent) {
    positionSetpoint = null;
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
  }
}
