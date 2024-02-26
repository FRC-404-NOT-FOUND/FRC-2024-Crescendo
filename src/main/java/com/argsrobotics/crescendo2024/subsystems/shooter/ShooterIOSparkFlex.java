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

package com.argsrobotics.crescendo2024.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ShooterIOSparkFlex implements ShooterIO {
  private final CANSparkFlex motor;
  private final RelativeEncoder encoder;
  private double percent = 0.0;

  public ShooterIOSparkFlex(int id) {
    motor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = motor.getEncoder();

    motor.restoreFactoryDefaults();

    motor.setInverted(false);
    motor.setSmartCurrentLimit(40);
    motor.enableVoltageCompensation(12.0);

    motor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.percent = percent;
    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();
    inputs.current = motor.getOutputCurrent();
    inputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
  }

  @Override
  public void setSpeed(double speeds) {
    percent = speeds;
    motor.set(speeds);
  }
}
