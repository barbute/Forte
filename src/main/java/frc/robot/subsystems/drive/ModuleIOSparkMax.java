// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** ModuleIO implementation for SparkMax motor controller (NEO) */
public class ModuleIOSparkMax implements ModuleIO {
  // TODO Set these for Forte's gear ratios
  private static final double DRIVE_GEAR_RATIO = 6.75 / 1.0;
  private static final double AZIMUTH_GEAR_RATIO = 150.0 / 7.0;

  private CANSparkMax driveMotor;
  private CANSparkMax azimuthMotor;

  private RelativeEncoder driveEncoder = driveMotor.getEncoder();
  private RelativeEncoder azimuthEncoder = azimuthMotor.getEncoder();
  private CANcoder azimuthAbsoluteEncoder;

  private Rotation2d absoluteEncoderOffset = new Rotation2d();

  private double driveAppliedVolts = 0.0;
  private double azimuthAppliedVolts = 0.0;

  public ModuleIOSparkMax(int index) {
    // TODO Update ID's, offsets, and CANBUS
    switch (index) {
      case 0:
        driveMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(0, "");

        absoluteEncoderOffset = new Rotation2d(0.0);
        break;
      case 1:
        driveMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(0, "");

        absoluteEncoderOffset = new Rotation2d(0.0);
        break;
      case 2:
        driveMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(0, "");

        absoluteEncoderOffset = new Rotation2d(0.0);
        break;
      case 3:
        driveMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthMotor = new CANSparkMax(0, MotorType.kBrushless);
        azimuthAbsoluteEncoder = new CANcoder(0, "");

        absoluteEncoderOffset = new Rotation2d(0.0);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveMotor.restoreFactoryDefaults();
    azimuthMotor.restoreFactoryDefaults();

    driveMotor.setCANTimeout(250);
    azimuthMotor.setCANTimeout(250);

    driveEncoder = driveMotor.getEncoder();
    azimuthEncoder = azimuthMotor.getEncoder();

    driveMotor.setInverted(false);
    azimuthMotor.setInverted(true);

    driveMotor.setSmartCurrentLimit(40);
    azimuthMotor.setSmartCurrentLimit(30);
    driveMotor.enableVoltageCompensation(12.0);
    azimuthMotor.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    azimuthEncoder.setPosition(0.0);
    azimuthEncoder.setMeasurementPeriod(10);
    azimuthEncoder.setAverageDepth(2);

    driveMotor.setIdleMode(IdleMode.kBrake);
    azimuthMotor.setIdleMode(IdleMode.kCoast);

    driveMotor.setCANTimeout(0);
    azimuthMotor.setCANTimeout(0);

    driveMotor.burnFlash();
    azimuthMotor.burnFlash();

    azimuthAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
    inputs.driveTemperatureCelsius = new double[] {driveMotor.getMotorTemperature()};

    inputs.azimuthAbsolutePosition =
        Rotation2d.fromRotations(azimuthAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(absoluteEncoderOffset);
    inputs.azimuthPosition =
        Rotation2d.fromRadians(
            Units.rotationsToRadians(azimuthEncoder.getPosition()) / AZIMUTH_GEAR_RATIO);
    inputs.azimuthVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(azimuthEncoder.getVelocity())
            / AZIMUTH_GEAR_RATIO;
    inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    inputs.azimuthCurrentAmps = new double[] {azimuthMotor.getOutputCurrent()};
    inputs.azimuthTemperatureCelsius = new double[] {azimuthMotor.getMotorTemperature()};
  }

  @Override
  public void setDriveVolts(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveMotor.setVoltage(driveAppliedVolts);
  }

  @Override
  public void setAzimuthVolts(double volts) {
    azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    azimuthMotor.setVoltage(azimuthAppliedVolts);
  }

  @Override
  public void setDriveBreak(boolean enableBrake) {
    if (enableBrake) {
      driveMotor.setIdleMode(IdleMode.kBrake);
    } else {
      driveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void setAzimuthBreak(boolean enableBrake) {
    if (enableBrake) {
      azimuthMotor.setIdleMode(IdleMode.kBrake);
    } else {
      azimuthMotor.setIdleMode(IdleMode.kCoast);
    }
  }
}
