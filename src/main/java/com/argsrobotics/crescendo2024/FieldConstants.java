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

package com.argsrobotics.crescendo2024;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
  public static AprilTagFieldLayout aprilTags =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // TODO: add Pose2ds for speaker, amp, source, climb lcr, starting pose, etc.
  public static Pose2d startingPoseLeft = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d startingPoseRight = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d startingPoseCenter = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  public static Pose2d speakerPodium = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d speakerSubwoofer = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d speakerLeftWing = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d speakerRightWing = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  // TODO: amp score path that scores while strafing for speed and consistency
  public static PathPlannerPath ampPath = null;
  // Starting point of the amp path
  public static Pose2d amp = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  public static Pose2d source = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d sourceTravel = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  public static Pose2d climbLeft = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d climbRight = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
  public static Pose2d climbCenter = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
}
