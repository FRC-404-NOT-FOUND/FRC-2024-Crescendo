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

import com.argsrobotics.crescendo2024.RobotState;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionCameraIOPhoton implements VisionCameraIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator estimator;
  private final Transform3d cameraTranslation;

  public VisionCameraIOPhoton(String cameraName, Transform3d cameraTranslation) {
    camera = new PhotonCamera(cameraName);
    estimator =
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            cameraTranslation);

    this.cameraTranslation = cameraTranslation;
  }

  @Override
  public void updateInputs(VisionCameraIOInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();

    inputs.connected = camera.isConnected();
    if (!inputs.connected) {
      inputs.mode = null;
      inputs.hasTarget = false;
      inputs.estimatedTargetPose = null;
      inputs.hasEstimatedPose = false;
      inputs.estimatedPose = null;
      inputs.timestamp = 0;
      return;
    }

    inputs.mode =
        camera.getPipelineIndex() == 0 ? VisionCameraIO.Mode.AprilTag : VisionCameraIO.Mode.Target;
    inputs.timestamp = result.getTimestampSeconds();

    if (inputs.mode == VisionCameraIO.Mode.AprilTag) {
      Optional<EstimatedRobotPose> pose = estimator.update(result);
      inputs.hasTarget = false;
      inputs.estimatedTargetPose = null;
      inputs.hasEstimatedPose = pose.isPresent();
      pose.ifPresent(p -> inputs.estimatedPose = p.estimatedPose.toPose2d());
    } else {
      inputs.hasTarget = result.hasTargets();
      inputs.estimatedTargetPose =
          new Pose3d(RobotState.getCurrentRobotState().currentPose)
              .transformBy(cameraTranslation)
              .transformBy(result.getBestTarget().getBestCameraToTarget())
              .toPose2d();
      inputs.hasEstimatedPose = false;
    }
  }

  @Override
  public void setMode(Mode mode) {
    switch (mode) {
      case AprilTag:
        camera.setPipelineIndex(0);
        break;
      case Target:
        camera.setPipelineIndex(1);
        break;
    }
  }

  @Override
  public Mode getMode() {
    if (camera.getPipelineIndex() == 0) {
      return Mode.AprilTag;
    } else {
      return Mode.Target;
    }
  }

  @Override
  public void close() {
    camera.close();
  }
}
