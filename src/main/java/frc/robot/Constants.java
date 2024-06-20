// Copyright (c) 2024 FRC 5411 & 9105

package frc.robot;

/**
 * Global constants class. Should only be used for final variables that must be in the global scope
 * of this project (don't be stupid).
 */
public class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final boolean debuggingMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
