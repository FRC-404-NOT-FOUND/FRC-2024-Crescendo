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

package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DriverOIDualFlightStick implements DriverOI {
    private final CommandJoystick left;
    private final CommandJoystick right;

    public DriverOIDualFlightStick(int lp, int rp) {
        left = new CommandJoystick(lp);
        right = new CommandJoystick(rp);
    }

    public DriverOIDualFlightStick() {
        this(0, 1);
    }

    @Override
    public double getDriveY() {
        return -left.getY();
    }

    @Override
    public double getDriveX() {
        return -left.getX();
    }

    @Override
    public double getDriveZ() {
        return -right.getZ();
    }

    @Override
    public double getCenterOfRotationX() {
        return right.getX();
    }

    @Override
    public double getCenterOfRotationY() {
        return right.getY();
    }
}
