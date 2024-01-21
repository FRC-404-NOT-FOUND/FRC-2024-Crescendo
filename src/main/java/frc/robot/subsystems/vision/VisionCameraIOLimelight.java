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

// My original plan was just to use PhotonVision for all of the cameras, including the Limelight.
// But this is backup in case PhotonVision doesn't work on the Limelight.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;

public class VisionCameraIOLimelight implements VisionCameraIO {
  private static NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");

  public VisionCameraIOLimelight(String cameraName, Transform3d cameraTranslation) {}

  @Override
  public void updateInputs(VisionCameraIOInputs inputs) {
    inputs.connected = nt.containsKey("tv");

    if (!inputs.connected) {
      inputs.mode = null;
      inputs.hasTarget = false;
      inputs.estimatedTargetPose = null;
      inputs.hasEstimatedPose = false;
      inputs.lastEstimatedPose = null;
      inputs.timestamp = 0;
      return;
    }

    inputs.mode = get("getpipe").getDouble(0) == 0 ? Mode.AprilTag : Mode.Target;
    inputs.timestamp =
        Timer.getFPGATimestamp() - ((get("cl").getDouble(0) + get("tl").getDouble(0)) / 1000.0);

    if (inputs.mode == Mode.AprilTag) {
      inputs.estimatedTargetPose = null;
      inputs.hasTarget = false;
      inputs.hasEstimatedPose = get("tid").getDouble(0) != 0;
      if (inputs.hasEstimatedPose) {
        double[] poseData = get("botpose").getDoubleArray(new double[6]);
        Pose3d pose =
            new Pose3d(
                new Translation3d(poseData[0], poseData[1], poseData[2]),
                new Rotation3d(poseData[3], poseData[4], poseData[5]));

        inputs.lastEstimatedPose = pose.toPose2d();
      }
    } else {
      inputs.lastEstimatedPose = null;
      inputs.hasEstimatedPose = false;
      inputs.hasTarget = get("tv").getDouble(0) != 0;
      if (inputs.hasTarget) {
        double[] poseData =
            get("targetpose_robotspace")
                .getDoubleArray(
                    new double[6]); // I sure hope this works for target tracking not just apriltags
        Transform3d transform =
            new Transform3d(
                new Translation3d(poseData[0], poseData[1], poseData[2]),
                new Rotation3d(poseData[3], poseData[4], poseData[5]));

        Pose3d pose =
            new Pose3d(RobotState.getCurrentRobotState().currentPose).transformBy(transform);
        inputs.estimatedTargetPose = pose.toPose2d();
      }
    }
  }

  @Override
  public void setMode(Mode mode) {
    switch (mode) {
      case AprilTag:
        get("pipeline").setDouble(0);
        break;
      case Target:
        get("pipeline").setDouble(1);
        break;
    }
  }

  @Override
  public Mode getMode() {
    if (get("getpipe").getDouble(0) == 0) {
      return Mode.AprilTag;
    } else {
      return Mode.Target;
    }
  }

  private static NetworkTableEntry get(String key) {
    return nt.getEntry(key);
  }
}
