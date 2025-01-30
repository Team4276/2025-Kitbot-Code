package frc.team4276.frc2025.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;

public class WheelRadiusCharacterization extends Command {
  private final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private final SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
  private WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

  private final Drive drive;

  /** Orient Modules before running characterization */
  public WheelRadiusCharacterization(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void initialize() {
    limiter.reset(0.0);

    state.positions = drive.getWheelRadiusCharacterizationPositions();
    state.lastAngle = RobotState.getInstance().getEstimatedPose().getRotation();
    state.gyroDelta = 0.0;
  }

  @Override
  public void execute() {
    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
    drive.runWheelRadiusCharacterization(speed);

    var rotation = RobotState.getInstance().getEstimatedPose().getRotation();
    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
    state.lastAngle = rotation;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    double[] positions = drive.getWheelRadiusCharacterizationPositions();
    double wheelDelta = 0.0;
    for (int i = 0; i < 4; i++) {
      wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
    }
    double wheelRadius = (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

    NumberFormat formatter = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization Results **********");
    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
    System.out.println(
        "\tWheel Radius: "
            + formatter.format(wheelRadius)
            + " meters, "
            + formatter.format(Units.metersToInches(wheelRadius))
            + " inches");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
