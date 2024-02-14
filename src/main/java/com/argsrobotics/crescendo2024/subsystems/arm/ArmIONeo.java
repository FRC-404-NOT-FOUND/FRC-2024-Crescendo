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
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ArmIONeo implements ArmIO {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private final RelativeEncoder encoder;
  private final SparkPIDController pidController;

  private Double positionSetpoint;

  public ArmIONeo(int left, int right) {
    leftMotor = new CANSparkMax(left, MotorType.kBrushless);
    rightMotor = new CANSparkMax(left, MotorType.kBrushless);

    leftMotor.follow(rightMotor);

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

    encoder = leftMotor.getEncoder();
    encoder.setPosition(0.0);
    encoder.setMeasurementPeriod(10);
    encoder.setAverageDepth(2);

    pidController = leftMotor.getPIDController();

    pidController.setP(kArmP.get());
    pidController.setI(kArmI.get());
    pidController.setD(kArmD.get());
    pidController.setFF(kArmFF.get());

    leftMotor.setCANTimeout(0);
    rightMotor.setCANTimeout(0);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (kTuningMode) {
      pidController.setP(kArmP.get());
      pidController.setI(kArmI.get());
      pidController.setD(kArmD.get());
      pidController.setFF(kArmFF.get());
    }

    if (positionSetpoint != null) {
      pidController.setReference(positionSetpoint, CANSparkBase.ControlType.kPosition);
    }

    inputs.setpoint = positionSetpoint;
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.current = leftMotor.getOutputCurrent();
    inputs.voltage = leftMotor.getBusVoltage();
  }

  @Override
  public void setPosition(double position) {
    positionSetpoint = position;
  }

  @Override
  public void setPercent(double percent) {
    leftMotor.set(percent);
    rightMotor.set(percent);
  }

  @Override
  public void close() {
    leftMotor.close();
    rightMotor.close();
  }
}
