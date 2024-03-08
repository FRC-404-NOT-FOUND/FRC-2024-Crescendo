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

package com.argsrobotics.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class CountingDelay {
  private final Timer timer = new Timer();

  public CountingDelay() {
    timer.start();
  }

  public boolean delay(double delay) {
    if (timer.get() >= delay) {
      return true;
    } else {
      return false;
    }
  }

  public void reset() {
    timer.reset();
    timer.start();
  }
}
