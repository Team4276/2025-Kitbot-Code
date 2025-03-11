package frc.team4276.util.dashboard;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.team4276.frc2025.Constants;

public class LoggedTunableProfiledPID extends ProfiledPIDController {
  public final LoggedTunableNumber Kp;
  public final LoggedTunableNumber Ki;
  public final LoggedTunableNumber Kd;
  public final LoggedTunableNumber KTol;
  public final LoggedTunableNumber maxVel;
  public final LoggedTunableNumber maxAccel;

  private final String key;

  public LoggedTunableProfiledPID(
      String key, double kp, double ki, double kd, double maxVel, double maxAccel) {
    super(kp, ki, kd, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    this.key = key;
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", getPositionTolerance());
    this.maxVel = new LoggedTunableNumber(this.key + "/MaxVel", maxVel);
    this.maxAccel = new LoggedTunableNumber(this.key + "/MaxAccel", maxAccel);
  }

  public LoggedTunableProfiledPID(
      String key, double kp, double ki, double kd, double tol, double maxVel, double maxAccel) {
    super(kp, ki, kd, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    this.key = key;
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", tol);
    this.maxVel = new LoggedTunableNumber(this.key + "/MaxVel", maxVel);
    this.maxAccel = new LoggedTunableNumber(this.key + "/MaxAccel", maxAccel);
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    if (Constants.isTuning) {
      setPID(Kp.getAsDouble(), Ki.getAsDouble(), Kd.getAsDouble());
      setTolerance(KTol.getAsDouble());
      setConstraints(
          new TrapezoidProfile.Constraints(maxVel.getAsDouble(), maxAccel.getAsDouble()));
    }

    return super.calculate(measurement, setpoint);
  }
}
