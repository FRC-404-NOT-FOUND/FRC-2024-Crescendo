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

import com.argsrobotics.crescendo2024.commands.DriveCommands;
// import com.argsrobotics.crescendo2024.commands.TuneDrivePID;
import com.argsrobotics.crescendo2024.oi.DriverOI;
import com.argsrobotics.crescendo2024.oi.DriverOIXBox;
import com.argsrobotics.crescendo2024.subsystems.arm.Arm;
import com.argsrobotics.crescendo2024.subsystems.arm.ArmIO;
import com.argsrobotics.crescendo2024.subsystems.arm.ArmIONeo;
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
  public final Arm arm;

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
        break;
    }

    RobotState.setDrivetrain(drive);
    RobotState.setVision(vision);
    RobotState.setArm(arm);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive SysId Characterization",
        drive.sysIdDriveMotorCommand()); // Try this instead of previous

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
            oi::getDriveY,
            oi::getDriveX,
            oi::getDriveZ,
            oi::getCenterOfRotationX,
            oi::getCenterOfRotationY));

    // arm.setDefaultCommand(new TuneArmPID(arm));

    oi.getArmUpEnabled().whileTrue(arm.setArmSpeed(oi::getArmUp));
    oi.getArmDownEnabled().whileTrue(arm.setArmSpeed(oi::getArmDown));
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
