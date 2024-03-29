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

// This file is mainly taken from LASA Robotics under the MIT License:
// https://github.com/lasarobotics/PurpleLib/blob/master/src/main/java/org/lasarobotics/drive/AdvancedSwerveKinematics.java

package com.argsrobotics.crescendo2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Advanced Swerve Kinematics
 *
 * <p>Provides functionality that can correct for second order kinematics
 */
public class AdvancedSwerveKinematics extends SwerveDriveKinematics {
  private static final double EPS = 1E-9;
  private static double ROBOT_LOOP_PERIOD;
  // private Translation2d[] m_moduleLocations;

  /**
   * Create a SecondOrderSwerveKinematics object
   *
   * <p>Corrects for path drift when the robot is rotating
   *
   * @param moduleLocations Location of all 4 swerve modules, LF/RF/LR/RR
   */
  public AdvancedSwerveKinematics(double loopPeriod, Translation2d... moduleLocations) {
    super(moduleLocations);
    if (moduleLocations.length < 2)
      throw new IllegalArgumentException("A swerve drive requires at least two modules");

    // m_moduleLocations = moduleLocations;
    ROBOT_LOOP_PERIOD = loopPeriod;
  }

  /**
   * Create a SecondOrderSwerveKinematics object
   *
   * <p>Corrects for path drift when the robot is rotating
   *
   * @param moduleLocations Location of all 4 swerve modules, LF/RF/LR/RR
   */
  public AdvancedSwerveKinematics(Translation2d... moduleLocations) {
    super(moduleLocations);
    if (moduleLocations.length < 2)
      throw new IllegalArgumentException("A swerve drive requires at least two modules");

    // m_moduleLocations = moduleLocations;
    ROBOT_LOOP_PERIOD = 0.02;
  }

  /**
   * Obtain a new pose from a constant curvature velocity
   *
   * @param delta Velocity
   * @return Pose
   */
  public static Pose2d exp(final Twist2d delta) {
    double sin_theta = Math.sin(delta.dtheta);
    double cos_theta = Math.cos(delta.dtheta);
    double s, c;
    if (Math.abs(delta.dtheta) < EPS) {
      s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
      c = .5 * delta.dtheta;
    } else {
      s = sin_theta / delta.dtheta;
      c = (1.0 - cos_theta) / delta.dtheta;
    }
    return new Pose2d(
        new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
        new Rotation2d(cos_theta, sin_theta));
  }

  /**
   * Obtain constant curvature velocity given pose
   *
   * @param transform Pose
   * @return Velocity
   */
  private static Twist2d log(final Pose2d transform) {
    final double dtheta = transform.getRotation().getRadians();
    final double half_dtheta = 0.5 * dtheta;
    final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
    double halftheta_by_tan_of_halfdtheta;
    if (Math.abs(cos_minus_one) < EPS)
      halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
    else
      halftheta_by_tan_of_halfdtheta =
          -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;

    final Translation2d translation_part =
        transform
            .getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
    return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
  }

  /**
   * Correct chassis speeds for second order kinematics
   *
   * @param requestedSpeeds Requested chassis speeds
   * @param driftRate Drift rate of the rotation (robot loop lookahead multiplier)
   * @return Corrected chassis speeds
   */
  public static ChassisSpeeds correctForDynamics(ChassisSpeeds requestedSpeeds, double driftRate) {
    Pose2d futureRobotPose =
        new Pose2d(
            requestedSpeeds.vxMetersPerSecond * ROBOT_LOOP_PERIOD,
            requestedSpeeds.vyMetersPerSecond * ROBOT_LOOP_PERIOD,
            Rotation2d.fromRadians(
                requestedSpeeds.omegaRadiansPerSecond * ROBOT_LOOP_PERIOD * driftRate));

    Twist2d twistForPose = log(futureRobotPose);

    ChassisSpeeds correctedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / ROBOT_LOOP_PERIOD,
            twistForPose.dy / ROBOT_LOOP_PERIOD,
            twistForPose.dtheta / ROBOT_LOOP_PERIOD);

    return correctedSpeeds;
  }

  /**
   * Correct chassis speeds for second order kinematics
   *
   * @param requestedSpeeds Requested chassis speeds
   * @return Corrected chassis speeds
   */
  public static ChassisSpeeds correctForDynamics(ChassisSpeeds requestedSpeeds) {
    return correctForDynamics(requestedSpeeds, 1.0);
  }

  /**
   * Convert chassis speed to states of individual modules using second order kinematics
   *
   * @param desiredSpeed Desired translation and rotation speed of the robot
   * @param centerOfRotation Center of rotation of the robot (for now this uses WPILib's default
   *     implementation)
   */
  @Override
  public SwerveModuleState[] toSwerveModuleStates(
      ChassisSpeeds desiredSpeed, Translation2d centerOfRotation) {
    // We need to calculate this every time so that it updates its internal state for when we
    // actually need it.
    var initialModuleStates = super.toSwerveModuleStates(desiredSpeed, centerOfRotation);
    return initialModuleStates;

    // // Use default implementation if there's a custom center of rotatio as we don't need to
    // // straighten out rotation.
    // if (centerOfRotation.getDistance(new Translation2d()) > 1e-4) {
    //   return initialModuleStates;
    // }

    // Matrix<N3, N1> firstOrderInputMatrix = new Matrix<>(N3(), N1());
    // Matrix<N2, N3> firstOrderMatrix = new Matrix<>(N2(), N3());
    // Matrix<N4, N1> secondOrderInputMatrix = new Matrix<>(N4(), N1());
    // Matrix<N2, N4> secondOrderMatrix = new Matrix<>(N2(), N4());
    // Matrix<N2, N2> rotationMatrix = new Matrix<>(N2(), N2());

    // firstOrderInputMatrix.set(0, 0, desiredSpeed.vxMetersPerSecond);
    // firstOrderInputMatrix.set(1, 0, desiredSpeed.vyMetersPerSecond);
    // firstOrderInputMatrix.set(2, 0, desiredSpeed.omegaRadiansPerSecond);

    // secondOrderInputMatrix.set(2, 0, Math.pow(desiredSpeed.omegaRadiansPerSecond, 2));

    // firstOrderMatrix.set(0, 0, 1);
    // firstOrderMatrix.set(1, 1, 1);

    // secondOrderMatrix.set(0, 0, 1);
    // secondOrderMatrix.set(1, 1, 1);

    // SwerveModuleState[] swerveModuleStates = new SwerveModuleState[m_moduleLocations.length];
    // double[] moduleTurnSpeeds = new double[m_moduleLocations.length];

    // for (int i = 0; i < m_moduleLocations.length; i++) {
    //   // Angle that the module location vector makes with respect to the robot
    //   Rotation2d moduleAngle =
    //       new Rotation2d(Math.atan2(m_moduleLocations[i].getY(), m_moduleLocations[i].getX()));
    //   // Angle that the module location vector makes with respect to the field for field centric
    // if
    //   // applicable
    //   moduleAngle = Rotation2d.fromRadians(moduleAngle.getRadians());
    //   double moduleX = m_moduleLocations[i].getNorm() * Math.cos(moduleAngle.getRadians());
    //   double moduleY = m_moduleLocations[i].getNorm() * Math.sin(moduleAngle.getRadians());
    //   // -r_y
    //   firstOrderMatrix.set(0, 2, -moduleY);
    //   // +r_x
    //   firstOrderMatrix.set(1, 2, +moduleX);

    //   Matrix<N2, N1> firstOrderOutput = firstOrderMatrix.times(firstOrderInputMatrix);

    //   double moduleHeading = Math.atan2(firstOrderOutput.get(1, 0), firstOrderOutput.get(0, 0));
    //   double moduleSpeed = Math.sqrt(firstOrderOutput.elementPower(2).elementSum());

    //   secondOrderMatrix.set(0, 2, -moduleX);
    //   secondOrderMatrix.set(0, 3, -moduleY);
    //   secondOrderMatrix.set(1, 2, -moduleY);
    //   secondOrderMatrix.set(1, 3, +moduleX);

    //   rotationMatrix.set(0, 0, +Math.cos(moduleHeading));
    //   rotationMatrix.set(0, 1, +Math.sin(moduleHeading));
    //   rotationMatrix.set(1, 0, -Math.sin(moduleHeading));
    //   rotationMatrix.set(1, 1, +Math.cos(moduleHeading));

    //   Matrix<N2, N1> secondOrderOutput =
    //       rotationMatrix.times(secondOrderMatrix.times(secondOrderInputMatrix));
    //   swerveModuleStates[i] =
    //       new SwerveModuleState(moduleSpeed, Rotation2d.fromRadians(moduleHeading));
    //   moduleTurnSpeeds[i] =
    //       secondOrderOutput.get(1, 0) / moduleSpeed - desiredSpeed.omegaRadiansPerSecond;
    // }

    // return swerveModuleStates;
  }
}
