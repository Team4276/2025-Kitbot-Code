package frc.team4276.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class BetterXboxController extends CommandXboxController {
  private double DEADBAND = 0.1;

  public BetterXboxController(int port) {
    super(port);
  }

  public BetterXboxController(int port, double deadband) {
    super(port);
    this.DEADBAND = deadband;
  }

  public void setDeadband(double deadband) {
    this.DEADBAND = deadband;
  }

  public class JoystickOutput {
    public final double x;
    public final double y;

    public JoystickOutput(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public JoystickOutput() {
      this.x = 0.0;
      this.y = 0.0;
    }
  }

  public JoystickOutput getRightWithDeadband() {
    return Math.hypot(getRightX(), getRightY()) < DEADBAND ? new JoystickOutput() : getRight();
  }

  public JoystickOutput getRight() {
    return new JoystickOutput(getRightX(), getRightY());
  }

  public JoystickOutput getLeftWithDeadband() {
    return Math.hypot(getLeftX(), getLeftY()) < DEADBAND ? new JoystickOutput() : getLeft();
  }

  public JoystickOutput getLeft() {
    return new JoystickOutput(getLeftX(), getLeftY());
  }
}
