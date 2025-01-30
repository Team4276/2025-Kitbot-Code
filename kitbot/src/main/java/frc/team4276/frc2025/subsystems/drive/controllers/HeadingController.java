package frc.team4276.frc2025.subsystems.drive.controllers;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.team4276.util.LoggedTunableNumber;
import java.util.function.Supplier;

public class HeadingController {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("HeadingController/kP", snapKp);
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("HeadingController/kI", snapKi);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("HeadingController/kD", snapKd);
  private static final LoggedTunableNumber toleranceDegrees =
      new LoggedTunableNumber("HeadingController/ToleranceDegrees", snapPositionTolerance);
  private PIDController controller;

  private Supplier<Rotation2d> targetHeadingSupplier;

  public HeadingController() {
    controller = new PIDController(kP.get(), kI.get(), kD.get());

    controller.setTolerance(Math.toRadians(toleranceDegrees.get()));
    controller.enableContinuousInput(-Math.PI, Math.PI);

    targetHeadingSupplier = () -> new Rotation2d();
  }

  public void setTarget(Supplier<Rotation2d> targetSupplier) {
    targetHeadingSupplier = targetSupplier;
  }

  public double update(double headingRadians) {
    // Update controller
    controller.setPID(kP.get(), kI.get(), kD.get());
    controller.setTolerance(Math.toRadians(toleranceDegrees.get()));

    double output = controller.calculate(headingRadians, targetHeadingSupplier.get().getRadians());

    return output;
  }
}
