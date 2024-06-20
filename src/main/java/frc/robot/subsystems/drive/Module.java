// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/** Class to represent a swerve module */
public class Module {
  private static final double WHEEL_RADIUS_METERS = 2.0 * Math.PI * (5.08 / 100);

  private ModuleIO moduleIO;
  private ModuleIOInputsAutoLogged moduleIOInputs = new ModuleIOInputsAutoLogged();
  private final int INDEX;

  private SimpleMotorFeedforward driveFeedforward;
  private PIDController driveFeedback;
  private PIDController azimuthFeedback;
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop

  private Rotation2d azimuthRelativeOffset = null; // Relative + Offset = Absolute

  private LoggedTunableNumber driveFeedbackP;
  private LoggedTunableNumber driveFeedbackI;
  private LoggedTunableNumber driveFeedbackD;

  private LoggedTunableNumber azimuthFeedbackP;
  private LoggedTunableNumber azimuthFeedbackI;
  private LoggedTunableNumber azimuthFeedbackD;

  public Module(ModuleIO io, int index) {
    moduleIO = io;
    INDEX = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        driveFeedback = new PIDController(0.1, 0.0, 0.0);
        azimuthFeedback = new PIDController(10.0, 0.0, 0.0);

        break;
      case SIM:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
        driveFeedback = new PIDController(0.1, 0.0, 0.0);
        azimuthFeedback = new PIDController(10.0, 0.0, 0.0);

        break;
      case REPLAY:
        driveFeedforward = new SimpleMotorFeedforward(0.1, 0.13);
        driveFeedback = new PIDController(0.05, 0.0, 0.0);
        azimuthFeedback = new PIDController(7.0, 0.0, 0.0);

        break;
      default:
        driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
        driveFeedback = new PIDController(0.0, 0.0, 0.0);
        azimuthFeedback = new PIDController(0.0, 0.0, 0.0);

        break;
    }

    driveFeedbackP = new LoggedTunableNumber("Drive/Tuning/DriveP", driveFeedback.getP());
    driveFeedbackI = new LoggedTunableNumber("Drive/Tuning/DriveI", driveFeedback.getI());
    driveFeedbackD = new LoggedTunableNumber("Drive/Tuning/DriveD", driveFeedback.getD());

    azimuthFeedbackP = new LoggedTunableNumber("Drive/Tuning/AzimuthP", azimuthFeedback.getP());
    azimuthFeedbackI = new LoggedTunableNumber("Drive/Tuning/AzimuthI", azimuthFeedback.getI());
    azimuthFeedbackD = new LoggedTunableNumber("Drive/Tuning/AzimuthD", azimuthFeedback.getD());

    azimuthFeedback.enableContinuousInput(-Math.PI, Math.PI);
    setBrakeMode(true);
  }

  /** Method called in subsystem periodic */
  public void peridoic() {
    moduleIO.updateInputs(moduleIOInputs);
    Logger.processInputs("Drive/Module" + Integer.toString(INDEX), moduleIOInputs);

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (azimuthRelativeOffset == null
        && moduleIOInputs.azimuthAbsolutePosition.getRadians() != 0.0) {
      azimuthRelativeOffset =
          moduleIOInputs.azimuthAbsolutePosition.minus(moduleIOInputs.azimuthPosition);
    }

    // Run closed loop turn control
    if (angleSetpoint != null) {
      moduleIO.setAzimuthVolts(
          // Must call getAngle() and not directly from inputs since getAngle() applies the offset
          azimuthFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

      Logger.recordOutput(
          "Drive/Module" + Integer.toString(INDEX) + "/AzimuthSetpoint",
          azimuthFeedback.getSetpoint());
      Logger.recordOutput(
          "Drive/Module" + Integer.toString(INDEX) + "/AzimuthError",
          azimuthFeedback.getPositionError());
      Logger.recordOutput(
          "Drive/Module" + Integer.toString(INDEX) + "/AzimuthOutput",
          azimuthFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

      // Run closed loop drive control
      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null) {
        // Scale velocity based on turn error
        //
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        double adjustSpeedSetpoint = speedSetpoint * Math.cos(azimuthFeedback.getPositionError());

        // Run drive controller
        double velocityRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS_METERS;
        moduleIO.setDriveVolts(
            driveFeedforward.calculate(velocityRadPerSec)
                + driveFeedback.calculate(
                    moduleIOInputs.driveVelocityRadPerSec, velocityRadPerSec));
      }
    }

    // Update controllers
    if (Constants.debuggingMode) {
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () -> setDrivePID(driveFeedbackP.get(), driveFeedbackI.get(), driveFeedbackD.get()),
          driveFeedbackP,
          driveFeedbackI,
          driveFeedbackD);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          () ->
              setAzimuthPID(azimuthFeedbackP.get(), azimuthFeedbackI.get(), azimuthFeedbackD.get()),
          azimuthFeedbackP,
          azimuthFeedbackI,
          azimuthFeedbackD);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    angleSetpoint = optimizedState.angle;
    speedSetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    moduleIO.setDriveVolts(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    moduleIO.setAzimuthVolts(0.0);
    moduleIO.setDriveVolts(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    moduleIO.setDriveBreak(enabled);
    moduleIO.setAzimuthBreak(enabled);
  }

  /** Sets the PID gains for drive feedback */
  private void setDrivePID(double kP, double kI, double kD) {
    driveFeedback.setP(kP);
    driveFeedback.setI(kI);
    driveFeedback.setD(kD);
  }

  /** Sets the PID gains for azimuth feedback */
  private void setAzimuthPID(double kP, double kI, double kD) {
    azimuthFeedback.setP(kP);
    azimuthFeedback.setI(kI);
    azimuthFeedback.setD(kD);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (azimuthRelativeOffset == null) {
      return new Rotation2d();
    } else {
      return moduleIOInputs.azimuthPosition.plus(azimuthRelativeOffset);
    }
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return moduleIOInputs.drivePositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return moduleIOInputs.driveVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return moduleIOInputs.driveVelocityRadPerSec;
  }
}
