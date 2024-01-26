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

package com.argsrobotics.crescendo2024.subsystems.drive;

import static com.argsrobotics.crescendo2024.Constants.Drive.kDirectionSlewRate;
import static com.argsrobotics.crescendo2024.Constants.Drive.kDrivebaseRadius;
import static com.argsrobotics.crescendo2024.Constants.Drive.kMagnitudeSlewRate;
import static com.argsrobotics.crescendo2024.Constants.Drive.kMaxAngularSpeed;
import static com.argsrobotics.crescendo2024.Constants.Drive.kMaxLinearSpeed;
import static com.argsrobotics.crescendo2024.Constants.Drive.kPathFollowLinearD;
import static com.argsrobotics.crescendo2024.Constants.Drive.kPathFollowLinearI;
import static com.argsrobotics.crescendo2024.Constants.Drive.kPathFollowLinearP;
import static com.argsrobotics.crescendo2024.Constants.Drive.kPathFollowRotationalD;
import static com.argsrobotics.crescendo2024.Constants.Drive.kPathFollowRotationalI;
import static com.argsrobotics.crescendo2024.Constants.Drive.kPathFollowRotationalP;
import static com.argsrobotics.crescendo2024.Constants.Drive.kRotationalSlewRate;
import static com.argsrobotics.crescendo2024.Constants.Drive.kTrackWidthX;
import static com.argsrobotics.crescendo2024.Constants.Drive.kTrackWidthY;

import com.argsrobotics.crescendo2024.RobotState;
import com.argsrobotics.crescendo2024.util.LocalADStarAK;
import com.argsrobotics.crescendo2024.util.SwerveUtils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * This is the class for the main drive subsystem. It doesn't really do much. Most of it is just
 * odometry code (as usual).
 */
public class Drive extends SubsystemBase implements AutoCloseable {
  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final SwerveDrivePoseEstimator estimator;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();
  private Rotation2d gyroOffset;

  private final SlewRateLimiter magLimiter = new SlewRateLimiter(kMagnitudeSlewRate);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotationalSlewRate);

  private double prevTime = WPIUtilJNI.now() * 1e-6;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private final Field2d field = new Field2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    updateOdometry();

    estimator =
        new SwerveDrivePoseEstimator(
            kinematics, gyroInputs.yawPosition, getModulePositions(), pose);
    gyroOffset = gyroInputs.yawPosition.minus(pose.getRotation());

    RobotState.getCurrentRobotState().currentPose = pose;
    RobotState.getCurrentRobotState().currentModuleStates = getModuleStates();

    // Configure AutoBuilder for PathPlanner
    // PathPlanner doesn't let you dynamically update PID values for tuning.
    // They're still in here as TunableNumbers just because I didn't feel like changing it.
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(
                kPathFollowLinearP.get(), kPathFollowLinearI.get(), kPathFollowLinearD.get()),
            new PIDConstants(
                kPathFollowRotationalP.get(),
                kPathFollowRotationalI.get(),
                kPathFollowRotationalD.get()),
            kMaxLinearSpeed,
            kDrivebaseRadius,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                ? DriverStation.getAlliance().get() == Alliance.Red
                : false,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          field.getObject("activePath").setPoses(activePath);
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          field.getObject("targetPose").setPose(targetPose);
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public void periodic() {
    updateOdometry();

    for (var module : modules) {
      module.periodic();
    }

    if (RobotBase.isReal() && DriverStation.isDisabled()) {
      // Stop moving when disabled
      for (var module : modules) {
        module.stop();
      }

      // Log emtpy setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // This is the default AdvantageKit implementation with no vision support. I've left it here
    // just in case
    // but we can easily use the SwerveDrivePoseEstimator to get the same job done with less work.
    //
    // // Update odometry
    // int deltaCount =
    //     gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    // for (int i = 0; i < 4; i++) {
    //   deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    // }
    // for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
    //   // Read wheel deltas from each module
    //   SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    //   for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
    //     wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
    //   }

    //   // The twist represents the motion of the robot since the last
    //   // sample in x, y, and theta based only on the modules, without
    //   // the gyro. The gyro is always disconnected in simulation.
    //   var twist = kinematics.toTwist2d(wheelDeltas);
    //   if (gyroInputs.connected) {
    //     // If the gyro is connected, replace the theta component of the twist
    //     // with the change in angle since the last sample.
    //     Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
    //     twist = new Twist2d(twist.dx, twist.dy,
    // gyroRotation.minus(lastGyroRotation).getRadians());
    //     lastGyroRotation = gyroRotation;
    //   }
    //   // Apply the twist (change since last sample) to the current pose
    //   pose = pose.exp(twist);
    // }

    int deltaCount =
        gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }

    Pose2d latestPose = pose;

    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      double timestamp = 0.0;
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        timestamp += modules[moduleIndex].getPositionDeltaTimestamps()[deltaIndex];
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // Get the average of the times of odometry samples.
      timestamp /= 4.0;

      Rotation2d rotation;
      Twist2d latestTwist = kinematics.toTwist2d(wheelDeltas);

      // Initial estimate of latest pose to use as a basis for rotation.
      latestPose = latestPose.exp(latestTwist);

      // We'll probabaly never need this but just in case and because AdvantageKit does it it
      // "theoretically" works without the gyro.
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        lastGyroRotation = gyroRotation;
        rotation = gyroRotation;
      } else {
        // If the gyro is not connected, get the latest pose and use that rotation.
        rotation = latestPose.getRotation().plus(gyroOffset);
      }

      // Because we're using deltas to update,
      // adding the timestampe will make it more accurate so it can get a better measurement of when
      // we were at that position since it uses a low-pass filter which is time-based.
      latestPose = estimator.updateWithTime(timestamp, rotation, wheelDeltas);
    }

    // Tecnically we could just do pose = latestPose but just in case a vision update happens we'll
    // do this. There's really no point though.
    pose = estimator.getEstimatedPosition();

    RobotState.getCurrentRobotState().currentPose = pose;
    RobotState.getCurrentRobotState().currentModuleStates = getModuleStates();

    // Update field
    field.setRobotPose(pose);
    SmartDashboard.putData(field);
  }

  /** Updates the odometry. */
  public void updateOdometry() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   * @param centerOfRot Center of rotation of the robot in meters
   * @param rateLimited Whether to use rate limiting
   */
  public void runVelocity(ChassisSpeeds speeds, Translation2d centerOfRot, boolean rateLimited) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    if (rateLimited) {
      discreteSpeeds = calculateSlewRate(discreteSpeeds);
    }

    SwerveModuleState[] setpointStates =
        kinematics.toSwerveModuleStates(discreteSpeeds, centerOfRot);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeed);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * Runs the drive at the desired velocity. This uses the default setting of zero center of
   * rotation difference and no rate limiting (for PathPlanner to automatically control
   * acceleration).
   *
   * @param speeds
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, new Translation2d(), false);
  }

  /** Calculate the applied slew rate based on the current ChassisSpeeds */
  public ChassisSpeeds calculateSlewRate(ChassisSpeeds speeds) {
    // Alright so I saw this in the Rev example code and wasn't sure why they were converting the
    // speeds to
    // polar coordinates until I tried to do it myself. Here's the explanation because I and
    // others will probably forget.
    // The way slew rate limiters work is by keeping track of the past values of a variable and
    // limiting the rate of change.
    // You cannot have a two variable rate limiter (well you could but whatever), so by
    // determining the polar coordinates of the speeds
    // (r and theta, distance and angle or in this context speed and direction),
    // You can later undo it using trigonometry but combine the variables into one so that you can
    // limit the rates equally as well as limiting direction change.

    // This is the omega direction in radians of the polar coordinates
    double translationDirection = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

    // This is the r magnitude of the polar coordinates
    double translationMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    double directionSlewRate;
    if (currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(kDirectionSlewRate / currentTranslationMag);
    } else {
      directionSlewRate =
          500.0; // some high number that means the slew rate is effectively instantaneous
    }

    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - prevTime;

    double angleDif = SwerveUtils.angleDifference(translationDirection, currentTranslationDir);

    if (angleDif < 0.45 * Math.PI) {
      currentTranslationDir =
          SwerveUtils.stepTowardsCircular(
              currentTranslationDir, translationDirection, directionSlewRate * elapsedTime);
      currentTranslationMag = magLimiter.calculate(translationMagnitude);
    } else if (angleDif > 0.85 * Math.PI) {
      // No change in direction, this is just to avoid floating-point errors
      if (currentTranslationMag > 1e-4) {
        currentTranslationMag = magLimiter.calculate(0.0);
      } else {
        currentTranslationDir = SwerveUtils.wrapAngle(currentTranslationDir + Math.PI);
        currentTranslationMag = magLimiter.calculate(translationMagnitude);
      }
    } else {
      currentTranslationDir =
          SwerveUtils.stepTowardsCircular(
              translationMagnitude, translationDirection, directionSlewRate * elapsedTime);
      currentTranslationMag = magLimiter.calculate(0.0);
    }

    prevTime = currentTime;

    speeds.vxMetersPerSecond = currentTranslationMag * Math.cos(currentTranslationDir);
    speeds.vyMetersPerSecond = currentTranslationMag * Math.sin(currentTranslationDir);
    speeds.omegaRadiansPerSecond = rotLimiter.calculate(speeds.omegaRadiansPerSecond);

    return speeds;
  }

  /** Pathfind to the starting point and then follow a path (PathPlanner or Choreo) */
  public Command followPath(String pathname) {
    // We want to prioritize Choreo trajectories because by nature they are more efficient
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathname);
    if (path == null) {
      path = PathPlannerPath.fromPathFile(pathname);
    }
    return AutoBuilder.pathfindThenFollowPath(
        path,
        new PathConstraints(
            kMaxLinearSpeed, kDirectionSlewRate, kMaxAngularSpeed, kRotationalSlewRate));
  }

  /** Pathfind to a specific point */
  public Command goToPoint(Pose2d point) {
    return AutoBuilder.pathfindToPose(
        point,
        new PathConstraints(
            kMaxLinearSpeed, kDirectionSlewRate, kMaxAngularSpeed, kRotationalSlewRate));
  }

  /** Follow an auto routine */
  public Command followAuto(String auto) {
    return AutoBuilder.buildAuto(auto);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/MeasuredPositions")
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the current ChassisSpeeds */
  @AutoLogOutput(key = "Odometry/ChassisSpeeds")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    estimator.resetPosition(lastGyroRotation, positions, pose);
    this.pose = pose;
    gyroOffset = lastGyroRotation.minus(pose.getRotation());
    RobotState.getCurrentRobotState().currentPose = pose;
  }

  /** Add vision measurements to the pose estimator. */
  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    estimator.addVisionMeasurement(pose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return kMaxLinearSpeed;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return kMaxAngularSpeed;
  }

  public void close() {
    SparkMaxOdometryThread.getInstance().close();
    gyroIO.close();
    for (var module : modules) {
      module.close();
    }
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(kTrackWidthX / 2.0, -kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, kTrackWidthY / 2.0),
      new Translation2d(-kTrackWidthX / 2.0, -kTrackWidthY / 2.0)
    };
  }
}
