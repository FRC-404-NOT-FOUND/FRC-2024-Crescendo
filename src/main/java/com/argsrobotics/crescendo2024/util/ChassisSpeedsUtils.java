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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtils {
  public static ChassisSpeeds discretize(ChassisSpeeds speeds, double dt, double driftRate) {
    var pose =
        new Pose2d(
            speeds.vxMetersPerSecond * dt,
            speeds.vyMetersPerSecond * dt,
            Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt * driftRate));
    var twist = new Pose2d().log(pose);
    return new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt);
  }
}
