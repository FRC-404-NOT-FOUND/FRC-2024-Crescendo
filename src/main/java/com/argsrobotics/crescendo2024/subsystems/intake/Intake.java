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

import com.argsrobotics.crescendo2024.RobotState;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Debouncer debounce = new Debouncer(0.2, DebounceType.kRising);

  public Intake(IntakeIO io) {
    this.io = io;

    io.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (edu.wpi.first.wpilibj.RobotState.isDisabled()) {
      stop();
    }

    Logger.processInputs("Intake", inputs);
    RobotState.getCurrentRobotState().intakeSpeed = inputs.percent;
  }

  public void intake() {
    io.setPercent(1);
  }

  public void spit() {
    io.setPercent(-1);
  }

  public void stop() {
    io.setPercent(0);
  }

  public Command intakeCommand() {
    return run(() -> intake())
        .until(() -> debounce.calculate(inputs.current > 10))
        .andThen(run(() -> io.setPercent(-0.2)).withTimeout(0.5))
        .andThen(this::stop);
  }

  public Command spitCommand() {
    return run(() -> spit()).withTimeout(3).andThen(this::stop);
  }
}
