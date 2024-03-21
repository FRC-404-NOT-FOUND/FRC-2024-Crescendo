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
import static com.argsrobotics.crescendo2024.Constants.Drive.kDriftRate;
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
import static com.argsrobotics.crescendo2024.Constants.kTuningMode;

import com.argsrobotics.crescendo2024.FieldConstants;
import com.argsrobotics.crescendo2024.RobotState;
import com.argsrobotics.crescendo2024.util.AdvancedSwerveKinematics;
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
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private AdvancedSwerveKinematics kinematics =
      new AdvancedSwerveKinematics(getModuleTranslations());
  private Pose2d pose = new Pose2d();

  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator estimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final SysIdRoutine driveRoutine;

  private final SysIdRoutine angleRoutine;

  private final PIDController headingPid =
      new PIDController(
          kPathFollowRotationalP.get(), kPathFollowRotationalI.get(), kPathFollowRotationalD.get());

  private final SlewRateLimiter magLimiter = new SlewRateLimiter(kMagnitudeSlewRate);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotationalSlewRate);

  private double prevTime = WPIUtilJNI.now() * 1e-6;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private double driftRate = kDriftRate.get();

  private Pose2d pointTowards = null;
  private Rotation2d desiredHeading = null;

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

    SparkMaxOdometryThread.getInstance().start();

    driveRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.per(Units.Second).of(2), Units.Volts.of(1), Units.Second.of(3)),
            new SysIdRoutine.Mechanism(this::runCharacterizationVolts, null, this));
    angleRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(this::runAngleCharacterization, null, this));

    // Configure AutoBuilder for PathPlanner
    // PathPlanner doesn't let you dynamically update PID values for tuning.
    // They're still in here as TunableNumbers just because I didn't feel like changing it.
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
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
        FieldConstants::shouldFlipPoint,
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
    if (kTuningMode) {
      driftRate = kDriftRate.get();
      headingPid.setPID(
          kPathFollowRotationalP.get(), kPathFollowRotationalI.get(), kPathFollowRotationalD.get());
    }

    processInputs();

    for (var module : modules) {
      module.periodic();
      // Logger.recordOutput(
      //     "SwerveStates/Module" + i + "Velocity", modules[i].getVelocityMetersPerSec());
      // Logger.recordOutput("SwerveStates/Module" + i + "Angle",
      // modules[i].getAngle().getDegrees());
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

    // double[] sampleTimestamps =
    //     modules[0].getOdometryTimestamps(); // All signals are sampled together
    // int sampleCount = sampleTimestamps.length;
    // for (int i = 0; i < sampleCount; i++) {
    //   // Read wheel positions and deltas from each module
    //   SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    //   SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    //   for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
    //     modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
    //     moduleDeltas[moduleIndex] =
    //         new SwerveModulePosition(
    //             modulePositions[moduleIndex].distanceMeters
    //                 - lastModulePositions[moduleIndex].distanceMeters,
    //             modulePositions[moduleIndex].angle);
    //     lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    //   }

    //   // Update gyro angle
    //   if (gyroInputs.connected) {
    //     // Use the real gyro angle
    //     rawGyroRotation = gyroInputs.odometryYawPositions[i];
    //   } else {
    //     // Use the angle delta from the kinematics and module deltas
    //     Twist2d twist = kinematics.toTwist2d(moduleDeltas);
    //     rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    //   }

    //   // Apply update
    //   estimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    // }

    SwerveModulePosition[] positions = getModulePositions();

    if (gyroInputs.connected) {
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      Twist2d twist = kinematics.toTwist2d(positions);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    estimator.update(rawGyroRotation, positions);

    pose = estimator.getEstimatedPosition();

    RobotState.getCurrentRobotState().currentPose = pose;
    RobotState.getCurrentRobotState().currentModuleStates = getModuleStates();

    if (pointTowards != null) {
      desiredHeading = pointTowards.relativeTo(pose).getRotation();
    }

    // Update field
    field.setRobotPose(pose);
    SmartDashboard.putData(field);
  }

  /** Process module and gyro inputs. */
  public void processInputs() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
  }

  /**
   * Run an exact setpoint <i>for tuning only</i> as it forces it to the exact angle instead of the
   * optimized one
   */
  public void runSetpoint(double velocity, Rotation2d angle) {
    for (var module : modules) {
      module.runSetpoint(new SwerveModuleState(velocity, angle), true);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   * @param centerOfRot Center of rotation of the robot in meters
   */
  public void runVelocity(ChassisSpeeds speeds, Translation2d centerOfRot) {
    // This is basically the same as ChassisSpeeds.discretize() but a little bit better (I hope).
    ChassisSpeeds discreteSpeeds = AdvancedSwerveKinematics.correctForDynamics(speeds, driftRate);

    if (desiredHeading != null) {
      discreteSpeeds.omegaRadiansPerSecond =
          headingPid.calculate(getHeading().getRadians(), desiredHeading.getRadians());
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
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    runVelocity(speeds, new Translation2d());
  }

  /** Calculate the applied slew rate based on the current ChassisSpeeds */
  public ChassisSpeeds calculateSlewRate(
      double translationDirection, double translationMagnitude, double omega) {
    // Alright so I saw this in the Rev example code and wasn't sure why they were converting the
    // speeds to polar coordinates until I tried to do it myself. Here's the explanation because I
    // and others will probably forget. The way slew rate limiters work is by keeping track of the
    // past values of a variable and limiting the rate of change. You cannot have a two variable
    // rate limiter (well you could but whatever), so by determining the polar coordinates of the
    // speeds (r and theta, distance and angle or in this context speed and direction), You can
    // later undo it using trigonometry but combine the variables into one so that you can limit the
    // rates equally as well as limiting direction change.

    double directionSlewRate;
    if (currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(kDirectionSlewRate / currentTranslationMag);
    } else {
      // some high number that means the slew rate is effectively instantaneous
      directionSlewRate = 500.0;
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

    ChassisSpeeds speeds = new ChassisSpeeds();
    speeds.vxMetersPerSecond =
        currentTranslationMag * Math.cos(currentTranslationDir) * getMaxLinearSpeedMetersPerSec();
    speeds.vyMetersPerSecond =
        currentTranslationMag * Math.sin(currentTranslationDir) * getMaxLinearSpeedMetersPerSec();
    speeds.omegaRadiansPerSecond = rotLimiter.calculate(omega) * getMaxAngularSpeedRadPerSec();

    return speeds;
  }

  /** Pathfind to the starting point and then follow a path (PathPlanner or Choreo) */
  public Command followPath(String pathname) {
    // We want to prioritize Choreo trajectories because by nature they are more efficient
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathname);
    if (path == null) {
      path = PathPlannerPath.fromPathFile(pathname);
    }

    return followPath(path);
  }

  /** Pathfind to the starting point and then follow a path (PathPlanner or Choreo) */
  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.pathfindThenFollowPath(
        path, new PathConstraints(kMaxLinearSpeed, 3.0, kMaxAngularSpeed, 3.0));
  }

  /** Pathfind to a specific point */
  public Command driveToPose(Pose2d point) {
    return AutoBuilder.pathfindToPoseFlipped(
        point, new PathConstraints(kMaxLinearSpeed, 3.0, kMaxAngularSpeed, 3.0));
  }

  /** Follow an auto routine */
  public Command followAuto(String auto) {
    return AutoBuilder.buildAuto(auto);
  }

  /** Rotate to a specific heading */
  public void setHeading(Rotation2d heading) {
    pointTowards = null;
    desiredHeading = heading;
  }

  /** Rotate towards a specific pose */
  public void setPose(Pose2d pose) {
    pointTowards = pose;
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

  /** Runs at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  public void runAngleCharacterization(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runAngleCharacterization(volts);
    }
  }

  /** Runs at the commanded voltage */
  public void runCharacterizationVolts(Measure<Voltage> volts) {
    runCharacterizationVolts(volts.in(Units.Volts));
  }

  public void runAngleCharacterization(Measure<Voltage> volts) {
    runAngleCharacterization(volts.in(Units.Volts));
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  public Command sysIdDriveMotorCommand() {
    return Commands.sequence(
        driveRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(3),
        driveRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(3),
        driveRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(3),
        driveRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command sysIdAngleMotorCommand() {
    return Commands.sequence(
        angleRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(3),
        angleRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(3),
        angleRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(3),
        angleRoutine.dynamic(SysIdRoutine.Direction.kReverse));
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
  public Rotation2d getHeading() {
    return pose.getRotation();
  }

  /** Resets the current odometry pose. */
  public void resetOdometry(Pose2d pose) {
    estimator.resetPosition(gyroInputs.yawPosition, getModulePositions(), pose);
    this.pose = pose;
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
