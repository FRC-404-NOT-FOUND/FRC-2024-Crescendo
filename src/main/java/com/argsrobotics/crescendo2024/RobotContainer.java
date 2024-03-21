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

import static com.argsrobotics.crescendo2024.Constants.Arm.kLeftMotor;
import static com.argsrobotics.crescendo2024.Constants.Arm.kRightMotor;
import static com.argsrobotics.crescendo2024.Constants.Drive.kBackLeftChassisAngularOffset;
import static com.argsrobotics.crescendo2024.Constants.Drive.kBackRightChassisAngularOffset;
import static com.argsrobotics.crescendo2024.Constants.Drive.kFrontLeftChassisAngularOffset;
import static com.argsrobotics.crescendo2024.Constants.Drive.kFrontRightChassisAngularOffset;
import static com.argsrobotics.crescendo2024.Constants.Intake.kIntakeMotor;
import static com.argsrobotics.crescendo2024.Constants.Shooter.kBottomLeftShooter;
import static com.argsrobotics.crescendo2024.Constants.Shooter.kBottomRightShooter;

import com.argsrobotics.crescendo2024.commands.DriveCommands;
import com.argsrobotics.crescendo2024.commands.auto.FourNoteCenter;
import com.argsrobotics.crescendo2024.commands.auto.ShootBackOut;
import com.argsrobotics.crescendo2024.commands.auto.StartAuto;
import com.argsrobotics.crescendo2024.commands.auto.StartAuto.Position;
// import com.argsrobotics.crescendo2024.commands.TuneDrivePID;
import com.argsrobotics.crescendo2024.commands.auto.ThreeNoteCenterLeft;
import com.argsrobotics.crescendo2024.commands.auto.ThreeNoteCenterRight;
import com.argsrobotics.crescendo2024.commands.auto.TwoNoteCenterCenter;
import com.argsrobotics.crescendo2024.commands.auto.TwoNoteCenterLeft;
import com.argsrobotics.crescendo2024.commands.auto.TwoNoteCenterRight;
import com.argsrobotics.crescendo2024.oi.DriverOI;
import com.argsrobotics.crescendo2024.oi.DriverOIXBox;
import com.argsrobotics.crescendo2024.subsystems.arm.Arm;
import com.argsrobotics.crescendo2024.subsystems.arm.Arm.ArmAngle;
import com.argsrobotics.crescendo2024.subsystems.arm.ArmIO;
import com.argsrobotics.crescendo2024.subsystems.arm.ArmIONeo;
import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import com.argsrobotics.crescendo2024.subsystems.drive.GyroIO;
import com.argsrobotics.crescendo2024.subsystems.drive.GyroIOADIS16448;
import com.argsrobotics.crescendo2024.subsystems.drive.ModuleIO;
import com.argsrobotics.crescendo2024.subsystems.drive.ModuleIOSim;
import com.argsrobotics.crescendo2024.subsystems.drive.ModuleIOSparkMax;
import com.argsrobotics.crescendo2024.subsystems.intake.Intake;
import com.argsrobotics.crescendo2024.subsystems.intake.IntakeIO;
import com.argsrobotics.crescendo2024.subsystems.intake.IntakeIONeo;
import com.argsrobotics.crescendo2024.subsystems.shooter.Shooter;
import com.argsrobotics.crescendo2024.subsystems.shooter.ShooterIO;
import com.argsrobotics.crescendo2024.subsystems.shooter.ShooterIO.ShooterSpeeds;
import com.argsrobotics.crescendo2024.subsystems.shooter.ShooterIOSparkMax;
import com.argsrobotics.crescendo2024.subsystems.vision.Vision;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Vision vision;
  public final Arm arm;
  public final Intake intake;
  public final Shooter shooter;

  private boolean useFieldRelative = false;
  private boolean useLimitSwitch = false;

  // Controller
  private final DriverOI oi = new DriverOIXBox();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOADIS16448(),
                new ModuleIOSparkMax(0, kFrontLeftChassisAngularOffset),
                new ModuleIOSparkMax(1, kFrontRightChassisAngularOffset),
                new ModuleIOSparkMax(2, kBackLeftChassisAngularOffset),
                new ModuleIOSparkMax(3, kBackRightChassisAngularOffset));

        vision =
            new Vision(
                new HashMap<>() {
                  {
                  }
                });

        arm = new Arm(new ArmIONeo(kLeftMotor, kRightMotor));

        intake = new Intake(new IntakeIONeo(kIntakeMotor));

        shooter =
            new Shooter(
                // new ShooterIOSparkFlex(kTopLeftShooter),
                // new ShooterIOSparkFlex(kTopRightShooter),
                new ShooterIOSparkMax(kBottomLeftShooter),
                new ShooterIOSparkMax(kBottomRightShooter));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                new HashMap<>() {
                  {
                  }
                });

        arm = new Arm(new ArmIO() {});

        intake = new Intake(new IntakeIO() {});

        shooter =
            new Shooter(
                new ShooterIO() {}, new ShooterIO() {}, new ShooterIO() {}, new ShooterIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                new HashMap<>() {
                  {
                  }
                });

        arm = new Arm(new ArmIO() {});

        intake = new Intake(new IntakeIO() {});

        shooter =
            new Shooter(
                new ShooterIO() {}, new ShooterIO() {}, new ShooterIO() {}, new ShooterIO() {});
        break;
    }

    RobotState.setDrivetrain(drive);
    RobotState.setVision(vision);
    RobotState.setArm(arm);
    RobotState.setIntake(intake);
    RobotState.setShooter(shooter);

    configureButtonBindings();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Left Nothing", new StartAuto(drive, arm, Position.LEFT));
    autoChooser.addOption("Center Nothing", new StartAuto(drive, arm, Position.CENTER));
    autoChooser.addOption("Right Nothing", new StartAuto(drive, arm, Position.RIGHT));

    autoChooser.addOption(
        "Left Back Out", new ShootBackOut(drive, arm, shooter, intake, Position.LEFT));
    autoChooser.addOption(
        "Center Back Out", new ShootBackOut(drive, arm, shooter, intake, Position.CENTER));
    autoChooser.addOption(
        "Right Back Out", new ShootBackOut(drive, arm, shooter, intake, Position.RIGHT));

    autoChooser.addOption("4 Note", new FourNoteCenter(drive, arm, intake, shooter));
    autoChooser.addOption("3 Note Left 2", new ThreeNoteCenterLeft(drive, arm, intake, shooter));
    autoChooser.addOption("3 Note Right 2", new ThreeNoteCenterRight(drive, arm, intake, shooter));
    autoChooser.addOption("2 Note Left", new TwoNoteCenterLeft(drive, arm, intake, shooter));
    autoChooser.addOption("2 Note Center", new TwoNoteCenterCenter(drive, arm, intake, shooter));
    autoChooser.addOption("2 Note Right", new TwoNoteCenterRight(drive, arm, intake, shooter));

    autoChooser.addOption(
        "Drive SysId Characterization",
        drive.sysIdDriveMotorCommand()); // Try this instead of previous
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            oi::getDriveY,
            oi::getDriveX,
            oi::getDriveZ,
            oi::getCenterOfRotationX,
            oi::getCenterOfRotationY,
            () -> {
              return useFieldRelative;
            }));

    // arm.setDefaultCommand(new TuneArmPID(arm));

    // oi.getArmUpEnabled().whileTrue(arm.setArmSpeed(oi::getArmUp));
    // oi.getArmDownEnabled().whileTrue(arm.setArmSpeed(oi::getArmDown));

    NamedCommands.registerCommand("start", new StartAuto(drive, arm, Position.CENTER));
    NamedCommands.registerCommand(
        "intake", intake.intakeCommand(shooter.feedBackwards(), () -> true));
    NamedCommands.registerCommand(
        "shoot",
        Commands.parallel(
            shooter.shoot(new ShooterSpeeds()),
            Commands.waitSeconds(0.5)
                .andThen(intake.feedCommand().withTimeout(0.5))
                .withTimeout(1)));
    NamedCommands.registerCommand(
        "outtake",
        Commands.parallel(intake.spitCommand(), shooter.shoot(new ShooterSpeeds(0.1, -0.1)))
            .withTimeout(0.4));

    oi.getArmDownPosition().onTrue(arm.setArmAngle(ArmAngle.INTAKE));
    oi.getArmAmpPosition().onTrue(arm.setArmAngle(ArmAngle.AMP));
    oi.getArmPodiumPosition().onTrue(arm.setArmAngle(ArmAngle.PODIUM));
    oi.getAmpShoot()
        .onTrue(
            Commands.parallel(
                    shooter.shoot(new ShooterSpeeds(-0.2, 0.2)).withTimeout(2),
                    Commands.waitSeconds(0.5).andThen(intake.feedCommand().withTimeout(1.5)))
                .andThen(arm.setArmAngle(ArmAngle.INTAKE)));

    oi.getIntake()
        .onTrue(
            intake.intakeCommand(
                shooter.feedBackwards(),
                () -> {
                  return useLimitSwitch;
                }));
    oi.getOuttake()
        .whileTrue(
            Commands.parallel(intake.spitCommand(), shooter.shoot(new ShooterSpeeds(0.1, -0.1))));
    oi.getShoot()
        .onTrue(
            Commands.parallel(
                    shooter.shoot(new ShooterSpeeds()).withTimeout(2.5),
                    Commands.waitSeconds(1).andThen(intake.feedCommand().withTimeout(1.5)))
                .andThen(arm.setArmAngle(ArmAngle.INTAKE)));

    oi.getToggleFieldOriented()
        .onTrue(
            Commands.runOnce(
                () -> {
                  useFieldRelative = !useFieldRelative;
                }));
    oi.getToggleLimitSwitch()
        .onTrue(
            Commands.runOnce(
                () -> {
                  useLimitSwitch = !useLimitSwitch;
                }));

    oi.getClimbDown().onTrue(arm.climbDown());
    // drive.setDefaultCommand(new TuneDrivePID(drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
