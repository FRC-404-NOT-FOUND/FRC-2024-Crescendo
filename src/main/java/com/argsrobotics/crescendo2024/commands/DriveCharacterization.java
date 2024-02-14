// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.argsrobotics.crescendo2024.commands;

import com.argsrobotics.crescendo2024.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.Logger;

public class DriveCharacterization extends SequentialCommandGroup {
  private final SysIdRoutine sysId;

  /** Creates a new DriveCharacterization. */
  public DriveCharacterization(Drive d) {
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Characterization/DriveState", state.toString())),
            new SysIdRoutine.Mechanism(d::runCharacterizationVolts, null, d));

    addRequirements(d);

    addCommands(
        sysId.quasistatic(Direction.kForward),
        Commands.waitSeconds(2),
        sysId.quasistatic(Direction.kReverse),
        Commands.waitSeconds(2),
        sysId.dynamic(Direction.kForward),
        Commands.waitSeconds(2),
        sysId.dynamic(Direction.kReverse));
  }
}
