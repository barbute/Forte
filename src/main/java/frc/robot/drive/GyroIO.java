// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface to represent a Gyroscope */
public interface GyroIO {
  @AutoLog
  /** Sensor data from the gyro - used as "inputs" for the robot code */
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
