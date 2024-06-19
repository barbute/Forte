// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of ModuleIO */
public class ModuleIOSim implements ModuleIO {
  private final double LOOP_PERIOD_S = 0.02;
  // TODO Set these for Forte's gear ratios
  private static final double DRIVE_GEAR_RATIO = 1.0 / 1.0;
  private static final double AZIMUTH_GEAR_RATIO = 1.0 / 1.0;

  private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), DRIVE_GEAR_RATIO, 0.025);
  private DCMotorSim azimuthMotor = new DCMotorSim(DCMotor.getNEO(1), AZIMUTH_GEAR_RATIO, 0.004);

  private final Rotation2d azimuthAbsoluteInitPosition =
      new Rotation2d(Math.random() * 2.0 * Math.PI);

  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveMotor.update(LOOP_PERIOD_S);
    azimuthMotor.update(LOOP_PERIOD_S);

    inputs.drivePositionRad = driveMotor.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveMotor.getCurrentDrawAmps())};
    inputs.driveTemperatureCelsius = new double[] {0.0};

    inputs.azimuthAbsolutePosition =
        new Rotation2d(azimuthMotor.getAngularPositionRad()).plus(azimuthAbsoluteInitPosition);
    inputs.azimuthPosition = Rotation2d.fromRadians(azimuthMotor.getAngularPositionRad());
    inputs.azimuthVelocityRadPerSec = azimuthMotor.getAngularVelocityRadPerSec();
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = new double[] {Math.abs(azimuthMotor.getCurrentDrawAmps())};
    inputs.azimuthTemperatureCelsius = new double[] {0.0};
  }

  @Override
  public void setDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthVolts(double volts) {
    azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    azimuthMotor.setInputVoltage(azimuthAppliedVolts);
  }
}
