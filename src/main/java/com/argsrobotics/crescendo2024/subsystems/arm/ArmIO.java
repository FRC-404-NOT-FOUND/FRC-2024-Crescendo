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

package com.argsrobotics.crescendo2024.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO extends AutoCloseable {
  @AutoLog
  public class ArmIOInputs {
    public double position;
    public double velocity;
    public double current;
    public double voltage;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(Double position) {}

  public default void setClimbAngle(Double position) {}

  public default void setPercent(double percent) {}

  public default void close() {}
}
