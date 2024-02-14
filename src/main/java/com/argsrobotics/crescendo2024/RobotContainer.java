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

import com.argsrobotics.crescendo2024.commands.DriveCharacterization;
import com.argsrobotics.crescendo2024.commands.DriveCommands;
import com.argsrobotics.crescendo2024.commands.FeedForwardCharacterization;
import com.argsrobotics.crescendo2024.oi.DriverOI;
import com.argsrobotics.crescendo2024.oi.DriverOIXBox;
import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import com.argsrobotics.crescendo2024.subsystems.drive.GyroIO;
import com.argsrobotics.crescendo2024.subsystems.drive.GyroIOADIS16448;
import com.argsrobotics.crescendo2024.subsystems.drive.ModuleIO;
import com.argsrobotics.crescendo2024.subsystems.drive.ModuleIOSim;
import com.argsrobotics.crescendo2024.subsystems.drive.ModuleIOSparkMax;
import com.argsrobotics.crescendo2024.subsystems.vision.Vision;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));

        vision =
            new Vision(
                new HashMap<>() {
                  {
                  }
                });
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
        break;
    }

    RobotState.setDrivetrain(drive);
    RobotState.setVision(vision);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    autoChooser.addOption(
        "Drive SysId Characterization",
        new DriveCharacterization(drive)); // Try this instead of previous

    // Configure the button bindings
    configureButtonBindings();
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
            oi::getDriveX,
            oi::getDriveY,
            oi::getDriveZ,
            oi::getCenterOfRotationX,
            oi::getCenterOfRotationY));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
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
