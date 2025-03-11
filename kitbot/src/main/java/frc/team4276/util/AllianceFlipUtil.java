package frc.team4276.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.field.FieldConstants;

public class AllianceFlipUtil {
  static {
    SmartDashboard.putBoolean("Sim/OverrideFlip", false);
  }

  private static boolean overrideFlip = true;

  public static double flipX(double x) {
    return FieldConstants.fieldLength - x;
  }

  public static double flipY(double y) {
    return FieldConstants.fieldWidth - y;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static double applyX(double x) {
    return shouldFlip() ? FieldConstants.fieldLength - x : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return shouldFlip()
        ? new Translation2d(applyX(translation.getX()), applyY(translation.getY()))
        : translation;
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? rotation.rotateBy(Rotation2d.kPi) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  /**
   * For SIM true sets to blue alliance
   *
   * @param shouldOverrideFlip
   */
  public static void overrideFlip(boolean shouldOverrideFlip) {
    overrideFlip = shouldOverrideFlip;
  }

  public static boolean shouldFlip() {
    overrideFlip = SmartDashboard.getBoolean("Sim/OverrideFlip", overrideFlip);

    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        && (Constants.isSim ? !overrideFlip : true);
  }
}
