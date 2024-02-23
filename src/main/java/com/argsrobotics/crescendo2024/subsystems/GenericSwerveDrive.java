package com.argsrobotics.crescendo2024.subsystems;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public interface GenericSwerveDrive {

    public default void runVelocity(ChassisSpeeds speeds) {
        runVelocity(speeds, new Translation2d());
    };

    public void runVelocity(ChassisSpeeds speeds, Translation2d centerOfRotation);

    public Command followPath(PathPlannerPath path);

    public Command followPath(String pathName);

    public Command driveToPose(Pose2d point);

    public Command followAuto(String auto);

    public Command sysIdDriveMotorCommand();

    public Command sysIdAngleMotorCommand();

    public void runSetpoint(double velocity, Rotation2d angle);

    public void stop();

    public void resetOdometry(Pose2d pose);

    public void addVisionMeasurement(Pose2d pose, double timestamp);

    public Pose2d getPose();

    public Rotation2d getHeading();

    public double getMaxLinearSpeedMetersPerSec();

    public double getMaxAngularSpeedRadPerSec();

    public ChassisSpeeds calculateSlewRate(double linearDirection, double linearMagnitude, double omega);

    public SubsystemBase getDriveSubsystem();
}
