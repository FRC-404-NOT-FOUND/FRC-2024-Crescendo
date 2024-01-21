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

package com.argsrobotics.crescendo2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.argsrobotics.crescendo2024.RobotState;
import com.argsrobotics.crescendo2024.subsystems.vision.VisionCameraIO.Mode;
import com.argsrobotics.crescendo2024.util.FilteredPose2d;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final String[] cameraNames;
  private final VisionCameraIO[] cameras;
  private final VisionCameraIOInputsAutoLogged[] cameraInputs;

  private FilteredPose2d filteredPose = new FilteredPose2d(); // Normalize target pose
  private Pose2d currentTarget = null;
  private boolean hasTarget = false;

  public Vision(HashMap<String, VisionCameraIO> cameras) {
    this.cameraNames = cameras.keySet().toArray(new String[0]);
    this.cameras = new VisionCameraIO[this.cameraNames.length];
    this.cameraInputs = new VisionCameraIOInputsAutoLogged[this.cameraNames.length];
    for (int i = 0; i < cameras.keySet().size(); i++) {
      this.cameras[i] = cameras.get(cameraNames[i]);
      this.cameraInputs[i] = new VisionCameraIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // We reset it here so that we can update the global robot state consistently based on the
    // calculations in periodic
    hasTarget = false;

    for (int i = 0; i < cameras.length; i++) {
      cameras[i].updateInputs(cameraInputs[i]);
      Logger.processInputs("Vision/Camera " + cameraNames[i], cameraInputs[i]);
    }

    for (var input : cameraInputs) {
      if (input.hasEstimatedPose) {
        RobotState.addVisionMeasurement(input.lastEstimatedPose, input.timestamp);
      } else if (input.hasTarget) {
        hasTarget = true;
        currentTarget = filteredPose.calculate(input.estimatedTargetPose);
        RobotState.getCurrentRobotState().currentTarget = input.estimatedTargetPose;
        RobotState.getCurrentRobotState().hasTarget = hasTarget;
      }
    }

    if (!hasTarget) {
      currentTarget = null;
      filteredPose.reset();
      RobotState.getCurrentRobotState().currentTarget = currentTarget;
      RobotState.getCurrentRobotState().hasTarget = hasTarget;
    }

    Mode[] modes = new Mode[cameras.length];
    for (int i = 0; i < cameras.length; i++) {
      modes[i] = cameras[i].getMode();
    }

    RobotState.getCurrentRobotState().currentCameraModes = modes;
  }

  /**
   * Sets the mode of the specified camera.
   *
   * @param name The name of the camera.
   * @param mode The mode to set.
   */
  public void setMode(String name, VisionCameraIO.Mode mode) {
    for (int i = 0; i < cameraNames.length; i++) {
      if (cameraNames[i].equals(name)) {
        cameras[i].setMode(mode);
      } else {
        // We only want one target camera at a time, the rest should be doing positioning
        if (mode == VisionCameraIO.Mode.Target) {
          cameras[i].setMode(VisionCameraIO.Mode.AprilTag);
        }
      }
    }
  }

  /**
   * Retrieves the mode of a camera with the specified name.
   *
   * @param name the name of the camera
   * @return the mode of the camera
   */
  public Mode getMode(String name) {
    for (int i = 0; i < cameraNames.length; i++) {
      if (cameraNames[i].equals(name)) {
        return cameras[i].getMode();
      }
    }
    return Mode.AprilTag;
  }

  /**
   * Retrieves the current target for the PhotonTrackedTarget class.
   *
   * @return the current target
   */
  public Pose2d getCurrentTarget() {
    return currentTarget;
  }
}
