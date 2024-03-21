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

package com.argsrobotics.crescendo2024.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverOIXBox implements DriverOI {
  private final CommandXboxController gamepad;

  public DriverOIXBox(int port) {
    gamepad = new CommandXboxController(port);
  }

  public DriverOIXBox() {
    this(0);
  }

  @Override
  public double getDriveY() {
    return -gamepad.getLeftY();
  }

  @Override
  public double getDriveX() {
    return -gamepad.getLeftX();
  }

  @Override
  public double getDriveZ() {
    return -gamepad.getRightX();
  }

  @Override
  public double getCenterOfRotationX() {
    return 0.0;
  }

  @Override
  public double getCenterOfRotationY() {
    return 0.0;
  }

  @Override
  public double getArmUp() {
    return gamepad.getRightTriggerAxis();
  }

  @Override
  public double getArmDown() {
    return -gamepad.getLeftTriggerAxis();
  }

  @Override
  public Trigger getArmUpEnabled() {
    return gamepad.rightTrigger(0.1);
  }

  @Override
  public Trigger getArmDownEnabled() {
    return gamepad.leftTrigger(0.1);
  }

  @Override
  public Trigger getIntake() {
    return gamepad.a();
  }

  @Override
  public Trigger getOuttake() {
    return gamepad.b();
  }

  @Override
  public Trigger getShoot() {
    return gamepad.x();
  }

  @Override
  public Trigger getArmDownPosition() {
    return gamepad.leftBumper();
  }

  @Override
  public Trigger getArmAmpPosition() {
    return gamepad.rightBumper();
  }

  @Override
  public Trigger getArmPodiumPosition() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger getAmpShoot() {
    return gamepad.y();
  }

  public Trigger getToggleLimitSwitch() {
    return gamepad.pov(90);
  }

  public Trigger getClimbDown() {
    return gamepad.pov(180);
  }

  public Trigger getToggleFieldOriented() {
    return gamepad.pov(270);
  }
}
