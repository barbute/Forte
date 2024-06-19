// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Interface to represent Swerve Module */
public interface ModuleIO {
  @AutoLog
  /** Sensor data from the module - used as "inputs" for the robot code */
  public static class ModuleIOInputs {
    public double drivePositionRad =
        0.0; // Represents angular position in Radians - the conversion to meters occurs in
    // Module.java
    // cause it makes it easier for simulation integration
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {0.0};
    public double[] driveTemperatureCelsius = new double[] {0.0};

    public Rotation2d azimuthAbsolutePosition = new Rotation2d();
    public Rotation2d azimuthPosition = new Rotation2d();
    public double azimuthVelocityRadPerSec = 0.0;
    public double azimuthAppliedVolts = 0.0;
    public double[] azimuthCurrentAmps = new double[] {0.0};
    public double[] azimuthTemperatureCelsius = new double[] {0.0};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVolts(double volts) {}

  /** Run the azimuth motor at the specified voltage. */
  public default void setAzimuthVolts(double volts) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBreak(boolean enableBrake) {}

  /** Enable or disable brake mode on the azimuth motor. */
  public default void setAzimuthBreak(boolean enableBrake) {}
}
