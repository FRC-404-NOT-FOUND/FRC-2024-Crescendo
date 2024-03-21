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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  // private final LinearFilter currentFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  // private double filteredCurrent = 0;
  // private final Debouncer debounce = new Debouncer(kDebounceDelay.get(), DebounceType.kRising);
  private final DigitalInput limitSwitch = new DigitalInput(0);

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
    // filteredCurrent = currentFilter.calculate(inputs.current);
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

  // @AutoLogOutput(key = "Intake/Current")
  // public double getCurrent() {
  //   return filteredCurrent;
  // }

  @AutoLogOutput(key = "Arm/LimitSwitch")
  public boolean limitSwitchTripped() {
    return limitSwitch.get();
  }

  public Command feedCommand() {
    return runEnd(() -> io.setPercent(1.0), () -> io.setPercent(0));
  }

  public Command intakeCommand(Command shooterCmd, BooleanSupplier useLimitSwitch) {
    return runEnd(() -> intake(), () -> io.setPercent(0))
        // .until(() -> debounce.calculate(filteredCurrent > kCurrentStop.get()))
        .until(() -> useLimitSwitch.getAsBoolean() && limitSwitchTripped())
        .andThen(
            Commands.parallel(runEnd(() -> io.setPercent(-0.2), () -> io.setPercent(0)), shooterCmd)
                .withTimeout(0.5));
  }

  public Command spitCommand() {
    return runEnd(this::spit, this::stop);
  }
}
