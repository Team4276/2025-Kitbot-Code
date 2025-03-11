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

  public static final boolean isTuning = true;

  public static final boolean isSim = mode == Mode.SIM;

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (getType() == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + getType());
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (getType() != RobotType.COMPBOT || isTuning) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
