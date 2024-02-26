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

package com.argsrobotics.crescendo2024.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIONeo implements IntakeIO {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private double percent = 0.0;

  public IntakeIONeo(int intake) {
    intakeMotor = new CANSparkMax(intake, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    intakeMotor.setSmartCurrentLimit(50);
    intakeMotor.enableVoltageCompensation(12.0);

    intakeEncoder.setPosition(0.0);
    intakeEncoder.setMeasurementPeriod(10);
    intakeEncoder.setAverageDepth(2);

    intakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.percent = percent;
    inputs.position = intakeEncoder.getPosition();
    inputs.velocity = intakeEncoder.getVelocity();
    inputs.voltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
    inputs.current = intakeMotor.getOutputCurrent();
  }

  @Override
  public void setPercent(double percent) {
    this.percent = percent;
    intakeMotor.set(percent);
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    intakeMotor.setIdleMode(enabled ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }
}
