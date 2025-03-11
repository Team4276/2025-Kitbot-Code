package frc.team4276.frc2025.field;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team4276.util.AllianceFlipUtil;

public class FieldConstants {
  public static final int aprilTagCount = 22;
  public static final AprilTagFieldLayout apriltagLayout =
      AprilTagFieldLayout // TODO: add fudge factors
          .loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final Translation2d fieldCenter =
      new Translation2d(Units.inchesToMeters(345.437979), Units.inchesToMeters(158.5));

  public static final double reefToFieldCenter = 4.284788;

  public static final Pose2d blueReefCenter =
      new Pose2d(fieldCenter.minus(new Translation2d(reefToFieldCenter, 0.0)), Rotation2d.kZero);

  public static final double alignOffset = Units.inchesToMeters(14.0);
  public static final double scoringOffset = Units.inchesToMeters(31.0);
  public static final double reefCenterToTag = Units.inchesToMeters(20.738196);
  public static final double tagToReef = Units.inchesToMeters(6.468853);

  public static final Translation2d reefToLeftAlign =
      new Translation2d(-1.0 * (reefCenterToTag + scoringOffset + alignOffset), tagToReef);
  public static final Translation2d reefToRightAlign =
      reefToLeftAlign.plus(new Translation2d(0.0, -2.0 * tagToReef));

  public static final Translation2d reefToLeftScore =
      new Translation2d(-1.0 * (reefCenterToTag + scoringOffset), tagToReef);
  public static final Translation2d reefToRightScore =
      reefToLeftScore.plus(new Translation2d(0.0, -2.0 * tagToReef));

  public static final Pose2d[] blueReefToScore = new Pose2d[12];
  public static final Pose2d[] blueReefToAlign = new Pose2d[12];

  static {
    for (int i = 0; i < 6; i++) {
      var angle = Rotation2d.fromDegrees(i * 60);
      blueReefToScore[i * 2] =
          blueReefCenter.plus(new Transform2d(reefToLeftScore.rotateBy(angle), angle));
      blueReefToAlign[i * 2] =
          blueReefCenter.plus(new Transform2d(reefToLeftAlign.rotateBy(angle), angle));
      blueReefToScore[i * 2 + 1] =
          blueReefCenter.plus(new Transform2d(reefToRightScore.rotateBy(angle), angle));
      blueReefToAlign[i * 2 + 1] =
          blueReefCenter.plus(new Transform2d(reefToRightAlign.rotateBy(angle), angle));
    }
  }

  public enum Reef {
    A(blueReefToScore[0], blueReefToAlign[0]),
    B(blueReefToScore[1], blueReefToAlign[1]),
    C(blueReefToScore[2], blueReefToAlign[2]),
    D(blueReefToScore[3], blueReefToAlign[3]),
    E(blueReefToScore[4], blueReefToAlign[4]),
    F(blueReefToScore[5], blueReefToAlign[5]),
    G(blueReefToScore[6], blueReefToAlign[6]),
    H(blueReefToScore[7], blueReefToAlign[7]),
    I(blueReefToScore[8], blueReefToAlign[8]),
    J(blueReefToScore[9], blueReefToAlign[9]),
    K(blueReefToScore[10], blueReefToAlign[10]),
    L(blueReefToScore[11], blueReefToAlign[11]);

    private final Pose2d score;
    private final Pose2d align;

    private Reef(Pose2d score, Pose2d align) {
      this.score = score;
      this.align = align;
    }

    public Pose2d getScore() {
      return AllianceFlipUtil.apply(score);
    }

    public Pose2d getBlueScore() {
      return score;
    }

    public Pose2d getRedScore() {
      return AllianceFlipUtil.flip(score);
    }

    public Pose2d getAlign() {
      return AllianceFlipUtil.apply(align);
    }

    public Pose2d getBlueAlign() {
      return align;
    }

    public Pose2d getRedAlign() {
      return AllianceFlipUtil.flip(align);
    }
  }
}
