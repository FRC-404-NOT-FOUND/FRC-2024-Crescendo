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

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean kTuningMode = true;

  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Drive {
    public static final double kDriveDeadband = 0.1;

    public static final double kTrackWidthY = Units.inchesToMeters(25.0);
    public static final double kTrackWidthX = Units.inchesToMeters(25.0);
    public static final double kDrivebaseRadius =
        Math.hypot(kTrackWidthX / 2.0, kTrackWidthY / 2.0);

    public static final double kMaxLinearSpeed = Units.feetToMeters(15.76);
    public static final double kMaxAngularSpeed = kMaxLinearSpeed / kDrivebaseRadius;

    public static final double kWheelRadius = Units.inchesToMeters(1.5);

    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveGearRatio = 4.71 / 1.0;
    public static final double kTurnGearRatio = 9424.0 / 203.0;

    // Slew Rate
    public static final double kMagnitudeSlewRate = 1.8 * kMaxLinearSpeed; // ft/sec^2
    public static final double kDirectionSlewRate = 1.2; // ft/sec^2
    public static final double kRotationalSlewRate = 2.0 * kMaxAngularSpeed; // ft/sec^2

    // PID/FF constants
    public static final TunableNumber kDriveS = new TunableNumber("Drive kS", 0.1);
    public static final TunableNumber kDriveV = new TunableNumber("Drive kV", 0.13);
    public static final TunableNumber kDriveP = new TunableNumber("Drive kP", 0.05);
    public static final TunableNumber kDriveI = new TunableNumber("Drive kI", 0.0);
    public static final TunableNumber kDriveD = new TunableNumber("Drive kD", 0.0);

    public static final TunableNumber kTurnP = new TunableNumber("Turn kP", 7.0);
    public static final TunableNumber kTurnI = new TunableNumber("Turn kI", 0.0);
    public static final TunableNumber kTurnD = new TunableNumber("Turn kD", 0.0);

    public static final TunableNumber kDriveSimS = new TunableNumber("Drive Sim kS", 0.0);
    public static final TunableNumber kDriveSimV = new TunableNumber("Drive Sim kV", 0.13);
    public static final TunableNumber kDriveSimP = new TunableNumber("Drive Sim kP", 0.1);
    public static final TunableNumber kDriveSimI = new TunableNumber("Drive Sim kI", 0.0);
    public static final TunableNumber kDriveSimD = new TunableNumber("Drive Sim kD", 0.0);

    public static final TunableNumber kTurnSimP = new TunableNumber("Turn Sim kP", 10.0);
    public static final TunableNumber kTurnSimI = new TunableNumber("Turn Sim kI", 0.0);
    public static final TunableNumber kTurnSimD = new TunableNumber("Turn Sim kD", 0.0);
  }
}
