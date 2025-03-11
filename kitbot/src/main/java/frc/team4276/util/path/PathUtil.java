package frc.team4276.util.path;

import static frc.team4276.frc2025.field.FieldConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathUtil {
  public static Translation2d mirrorLengthwise(Translation2d trans) {
    return new Translation2d(trans.getX(), fieldWidth - trans.getY());
  }

  public static Rotation2d mirrorLengthwise(Rotation2d trans) {
    return trans.unaryMinus();
  }

  public static Pose2d mirrorLengthwise(Pose2d pose) {
    return new Pose2d(
        mirrorLengthwise(pose.getTranslation()), mirrorLengthwise(pose.getRotation()));
  }

  public static ChassisSpeeds mirrorLengthwise(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond,
        -1.0 * speeds.vyMetersPerSecond,
        -1.0 * speeds.omegaRadiansPerSecond);
  }
}
