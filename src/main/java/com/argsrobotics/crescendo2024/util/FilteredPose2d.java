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

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/* Normalize a Pose2d using a Single Pole IIR filter. This can be used to dampen noise in vision target pose readings. */
public class FilteredPose2d {
  private LinearFilter[] filters = new LinearFilter[3];

  public FilteredPose2d() {
    for (int i = 0; i < filters.length; i++) {
      filters[i] = LinearFilter.singlePoleIIR(0.1, 0.02);
    }
  }

  /**
   * Calculate the normalized Pose2d based on the input Pose2d.
   *
   * @param pose the input Pose2d
   * @return the normalized Pose2d
   */
  public Pose2d calculate(Pose2d pose) {
    return new Pose2d(
        filters[0].calculate(pose.getX()),
        filters[1].calculate(pose.getY()),
        Rotation2d.fromDegrees(filters[2].calculate(pose.getRotation().getDegrees())));
  }

  /** Resets the filter. */
  public void reset() {
    for (int i = 0; i < filters.length; i++) {
      filters[i].reset();
    }
  }
}
