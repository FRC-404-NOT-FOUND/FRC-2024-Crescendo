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

import com.argsrobotics.crescendo2024.FieldConstants;
import com.argsrobotics.crescendo2024.RobotState;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionCameraIOPhotonSim implements VisionCameraIO {
  private static final VisionSystemSim sim = new VisionSystemSim("main");
  private final SimCameraProperties properties = new SimCameraProperties();
  // private final Transform3d cameraTranslation;
  private PhotonCamera camera;
  private PhotonCameraSim simCamera;
  private PhotonPoseEstimator estimator;

  public VisionCameraIOPhotonSim(String cameraName, Transform3d cameraTranslation) {
    // this.cameraTranslation = cameraTranslation;

    sim.addAprilTags(FieldConstants.aprilTags);
    camera = new PhotonCamera(cameraName);
    simCamera = new PhotonCameraSim(camera, properties);
    sim.addCamera(simCamera, cameraTranslation);
    estimator =
        new PhotonPoseEstimator(
            FieldConstants.aprilTags,
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            camera,
            cameraTranslation);
  }

  @Override
  public void updateInputs(VisionCameraIOInputs inputs) {
    sim.update(RobotState.getCurrentRobotState().currentPose);

    PhotonPipelineResult result = camera.getLatestResult();
    estimator.setReferencePose(RobotState.getCurrentRobotState().currentPose);
    estimator.update(result);

    inputs.connected = true;
    inputs.mode = Mode.AprilTag;
    inputs.hasTarget = false;
    inputs.estimatedTargetPose = null;
    inputs.hasEstimatedPose = false;
    inputs.estimatedPose = null;
    inputs.timestamp = 0;
  }

  @Override
  public void setMode(Mode mode) {}

  @Override
  public Mode getMode() {
    return Mode.AprilTag;
  }

  @Override
  public void close() {
    camera.close();
    simCamera.close();
  }
}
