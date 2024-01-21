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

package com.argsrobotics.crescendo2024.util;

import com.argsrobotics.crescendo2024.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 *
 * <p>Thanks to team 422 for this idea (and code).
 */
public class TunableNumber {
  private static final String tableKey = "TunableNumbers";

  private String key;
  private double defaultValue;
  private double lastHasChangedValue = defaultValue;

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.kTuningMode) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    } else {
      SmartDashboard.clearPersistent(key);
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  public double get() {
    return Constants.kTuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }
}
