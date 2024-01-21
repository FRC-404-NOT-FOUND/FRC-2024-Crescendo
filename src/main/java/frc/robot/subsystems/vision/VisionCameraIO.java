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

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionCameraIO {
    public enum Mode {
        AprilTag,
        Target
    }

    @AutoLog
    public static class VisionCameraIOInputs {
        public boolean connected;
        public Mode mode;
        public boolean hasTarget;
        public boolean hasEstimatedPose;
        public Pose2d lastEstimatedPose;
        public Pose2d estimatedTargetPose;
        public double timestamp;
    }

    /** Updates the inputs for the camera pose and targets. */
    public default void updateInputs(VisionCameraIOInputs inputs) {}

    /** Sets the camera mode. */
    public default void setMode(Mode mode) {}

    /** Gets the camera mode. */
    public default Mode getMode() {
        return Mode.AprilTag;
    }
}
