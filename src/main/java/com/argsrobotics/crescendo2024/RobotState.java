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

import com.argsrobotics.crescendo2024.subsystems.arm.Arm;
import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import com.argsrobotics.crescendo2024.subsystems.intake.Intake;
import com.argsrobotics.crescendo2024.subsystems.shooter.Shooter;
import com.argsrobotics.crescendo2024.subsystems.shooter.ShooterIO.ShooterSpeeds;
import com.argsrobotics.crescendo2024.subsystems.vision.Vision;
import com.argsrobotics.crescendo2024.subsystems.vision.VisionCameraIO.Mode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class RobotState {
  private static RobotState currentRobotState = new RobotState();

  private static Drive drivetrain;
  private static Vision vision;
  private static Arm arm;
  private static Intake intake;
  private static Shooter shooter;

  public Pose2d currentPose = null;
  public SwerveModuleState[] currentModuleStates = null;

  public Mode[] currentCameraModes = null;
  public Pose2d currentTarget = null;
  public boolean hasTarget = false;

  public Rotation2d armAngle = null;

  public double intakeSpeed = 0.0;

  public ShooterSpeeds shooterSpeeds = new ShooterSpeeds();

  public RobotState() {}

  public static void addVisionMeasurement(Pose2d pose, double timestamp) {
    drivetrain.addVisionMeasurement(pose, timestamp);
  }

  public static Drive getDrivetrain() {
    return drivetrain;
  }

  public static Vision getVision() {
    return vision;
  }

  public static Arm getArm() {
    return arm;
  }

  public static Intake getIntake() {
    return intake;
  }

  public static Shooter getShooter() {
    return shooter;
  }

  public static void setDrivetrain(Drive drivetrain) {
    RobotState.drivetrain = drivetrain;
  }

  public static void setVision(Vision vision) {
    RobotState.vision = vision;
  }

  public static void setArm(Arm arm) {
    RobotState.arm = arm;
  }

  public static void setIntake(Intake intake) {
    RobotState.intake = intake;
  }

  public static void setShooter(Shooter shooter) {
    RobotState.shooter = shooter;
  }

  public static RobotState getCurrentRobotState() {
    return currentRobotState;
  }
}
