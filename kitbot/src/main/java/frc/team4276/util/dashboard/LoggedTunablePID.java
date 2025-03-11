package frc.team4276.util.dashboard;

import edu.wpi.first.math.controller.PIDController;
import frc.team4276.frc2025.Constants;

public class LoggedTunablePID extends PIDController {
  public final LoggedTunableNumber Kp;
  public final LoggedTunableNumber Ki;
  public final LoggedTunableNumber Kd;
  public final LoggedTunableNumber KTol;

  private final String key;

  public LoggedTunablePID(double kp, double ki, double kd, String key) {
    super(kp, ki, kd);
    this.key = key;
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", getErrorTolerance());
  }

  public LoggedTunablePID(double kp, double ki, double kd, double tol, String key) {
    super(kp, ki, kd);
    this.key = key;
    Kp = new LoggedTunableNumber(this.key + "/kP", kp);
    Ki = new LoggedTunableNumber(this.key + "/kI", ki);
    Kd = new LoggedTunableNumber(this.key + "/kD", kd);
    KTol = new LoggedTunableNumber(this.key + "/Tolerance", tol);
  }

  @Override
  public double calculate(double measurement, double setpoint) {
    if (Constants.isTuning) {
      setPID(Kp.getAsDouble(), Ki.getAsDouble(), Kd.getAsDouble());
      setTolerance(KTol.getAsDouble());
    }
    return super.calculate(measurement, setpoint);
  }
}
