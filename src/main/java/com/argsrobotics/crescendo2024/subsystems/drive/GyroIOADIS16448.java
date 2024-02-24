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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.function.Supplier;

/** IO implementation for ADIS16448 IMU */
public class GyroIOADIS16448 implements GyroIO {
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  private final Supplier<OptionalDouble> yaw =
      () -> {
        if (gyro.isConnected()) {
          return OptionalDouble.of(gyro.getAngle());
        } else {
          return OptionalDouble.empty();
        }
      };
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOADIS16448() {
    gyro.reset();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(yaw);
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(gyro.getGyroAngleZ());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getGyroRateZ());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void close() {
    gyro.close();
  }
}
