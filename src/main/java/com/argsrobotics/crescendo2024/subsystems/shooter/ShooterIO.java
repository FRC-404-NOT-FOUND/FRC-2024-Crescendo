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

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  public static class ShooterSpeeds {
    // public double tl;
    // public double tr;
    public double top;
    public double bottom;

    public ShooterSpeeds(double t, double b) {
      // tl = topLeft;
      // tr = topRight;
      top = t;
      bottom = b;
    }

    public ShooterSpeeds() {
      // tl = -0.5;
      // tr = 0.5;
      top = -0.5;
      bottom = 0.5;
    }

    public void setIndex(int index, double speed) {
      switch (index) {
          // case 0:
          //   tl = speed;
          //   break;
          // case 1:
          //   tr = speed;
          //   break;
        case 0:
          top = speed;
          break;
        case 1:
          bottom = speed;
          break;
      }
    }

    public double getIndex(int index) {
      switch (index) {
          // case 0:
          //   return tl;
          // case 1:
          //   return tr;
        case 0:
          return top;
        case 1:
          return bottom;
        default:
          return 0;
      }
    }
  }

  @AutoLog
  public static class ShooterIOInputs {
    public double percent;
    public double position;
    public double velocity;
    public double current;
    public double voltage;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setSpeed(double speeds) {}
}
