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

package frc.robot.subsystems.drive;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

/** IO implementation for ADIS16448 IMU */
public class GyroIOADIS16448 implements GyroIO {
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  private final DoubleSupplier yaw = () -> gyro.getGyroAngleZ();
  private final Queue<Double> yawPositionQueue;

  public GyroIOADIS16448() {
    gyro.reset();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(yaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(gyro.getGyroAngleZ());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getGyroRateZ());

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawPositionQueue.clear();
  }
}
