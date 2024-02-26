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

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double percent = 0.0;
    public double position = 0.0;
    public double velocity = 0.0;
    public double voltage = 0.0;
    public double current = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void setBrakeMode(boolean enabled) {}
}
