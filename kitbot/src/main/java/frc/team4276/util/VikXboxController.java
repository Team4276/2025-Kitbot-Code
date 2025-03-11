package frc.team4276.util;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class VikXboxController extends CommandXboxController {
  private double JOYSTICK_DEADBAND = 0.1;
  private double TRIGGER_DEADBAND = 0.5;

  public VikXboxController(int port) {
    super(port);
  }

  public VikXboxController(int port, double deadband) {
    super(port);
    this.JOYSTICK_DEADBAND = deadband;
  }

  public void setDeadband(double deadband) {
    this.JOYSTICK_DEADBAND = deadband;
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
    return Math.hypot(getRightX(), getRightY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getRight();
  }

  public JoystickOutput getRight() {
    return new JoystickOutput(getRightX(), getRightY());
  }

  public JoystickOutput getLeftWithDeadband() {
    return Math.hypot(getLeftX(), getLeftY()) < JOYSTICK_DEADBAND
        ? new JoystickOutput()
        : getLeft();
  }

  public JoystickOutput getLeft() {
    return new JoystickOutput(getLeftX(), getLeftY());
  }

  public boolean getPOVUP() {
    return getHID().getPOV() == 0;
  }

  public boolean getPOVRIGHT() {
    return getHID().getPOV() == 90;
  }

  public boolean getPOVDOWN() {
    return getHID().getPOV() == 180;
  }

  public boolean getPOVLEFT() {
    return getHID().getPOV() == 270;
  }

  public boolean getLT() {
    return getLeftTriggerAxis() > TRIGGER_DEADBAND;
  }

  public boolean getRT() {
    return getRightTriggerAxis() > TRIGGER_DEADBAND;
  }

  public Command rumbleCommand(RumbleType type, double value, double duration) {
    return rumbleCommand(type, value, duration, 1);
  }

  public Command rumbleCommand(RumbleType type, double value, double duration, int times) {
    var command = new SequentialCommandGroup();

    for (int i = 0; i < times; i++) {
      command.addCommands(
          Commands.startEnd(() -> setRumble(type, value), () -> setRumble(type, 0.0))
              .withTimeout(duration)
              .andThen(Commands.waitSeconds(0.1)));
    }

    return command;
  }
}
