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

import com.argsrobotics.crescendo2024.RobotState;
import com.argsrobotics.crescendo2024.subsystems.shooter.ShooterIO.ShooterSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO[] motors;
  private final ShooterIOInputsAutoLogged[] inputs;

  public Shooter(ShooterIO... ios) {
    motors = ios;
    inputs = new ShooterIOInputsAutoLogged[ios.length];

    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new ShooterIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < motors.length; i++) {
      motors[i].updateInputs(inputs[i]);
      Logger.processInputs("Shooter/Motor" + (i + 1), inputs[i]);
    }

    if (edu.wpi.first.wpilibj.RobotState.isDisabled()) {
      stop();
    }

    RobotState.getCurrentRobotState().shooterSpeeds = getSpeeds();
  }

  public void stop() {
    for (int i = 0; i < motors.length; i++) {
      motors[i].setSpeed(0);
    }
  }

  public void setSpeeds(ShooterSpeeds speeds) {
    for (int i = 0; i < motors.length; i++) {
      motors[i].setSpeed(speeds.getIndex(i));
    }
  }

  @AutoLogOutput(key = "Shooter/Speeds")
  public ShooterSpeeds getSpeeds() {
    ShooterSpeeds speeds = new ShooterSpeeds();
    for (int i = 0; i < motors.length; i++) {
      speeds.setIndex(i, inputs[i].percent);
    }

    return speeds;
  }

  public Command shoot(ShooterSpeeds speeds) {
    return run(() -> setSpeeds(speeds)).withTimeout(3).andThen(this::stop);
  }
}
