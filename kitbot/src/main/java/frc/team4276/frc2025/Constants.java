// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2025;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
@SuppressWarnings("unused")
public final class Constants {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY,
  }

  public static Mode getMode() {
    return mode;
  }

  public static enum RobotType {
    COMPBOT,
    SIMBOT
  }

  public static Mode mode = Mode.REAL;

  public static RobotType getType() {
    return switch (mode) {
      case REAL -> RobotType.COMPBOT;
      case REPLAY -> RobotType.COMPBOT;
      case SIM -> RobotType.SIMBOT;
    };
  }

  public static final boolean SysIdMode = false;

  static {
    assert !(SysIdMode && getMode() != Mode.REAL)
        : "Robot must be in REAL mode when SysIdMode is enabled.";
  }

  public static final boolean isTuning = false;
}
