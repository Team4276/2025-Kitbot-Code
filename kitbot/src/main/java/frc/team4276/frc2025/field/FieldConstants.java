package frc.team4276.frc2025.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875958);
  public static final double fieldWidth = Units.inchesToMeters(317.000000);
  public static final Translation2d fieldCenter = new Translation2d(
      Units.inchesToMeters(345.437979),
      Units.inchesToMeters(158.5));

  public static class POIs {
    public Translation2d reefCenter = new Translation2d();
    // starts at the right most post just under the 0 degree line and moves
    // counterclockwise around the reef
    public Pose2d[] reefScoring = new Pose2d[12];
  }

  public static final double scoringOffset = 40; // inches

  public static final Translation2d reefToLeftScoring = new Translation2d(
      Units.inchesToMeters(20.738196 + scoringOffset), Units.inchesToMeters(-6.468853));
  public static final Translation2d reefToRightScoring = new Translation2d(
      Units.inchesToMeters(20.738196 + scoringOffset), Units.inchesToMeters(6.468853));

  public static final POIs bluePOIs = new POIs();
  static {
    bluePOIs.reefCenter = fieldCenter.plus(new Translation2d(-4.284788, 0.0));
    bluePOIs.reefScoring[0] = new Pose2d(bluePOIs.reefCenter.plus(reefToLeftScoring), Rotation2d.fromDegrees(180.0));
    bluePOIs.reefScoring[1] = new Pose2d(bluePOIs.reefCenter.plus(reefToRightScoring), Rotation2d.fromDegrees(180.0));
    bluePOIs.reefScoring[2] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(60.0))),
        Rotation2d.fromDegrees(240.0));
    bluePOIs.reefScoring[3] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(60.0))),
        Rotation2d.fromDegrees(240.0));
    bluePOIs.reefScoring[4] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(120.0))),
        Rotation2d.fromDegrees(300.0));
    bluePOIs.reefScoring[5] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(120.0))),
        Rotation2d.fromDegrees(300.0));
    bluePOIs.reefScoring[6] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(180.0))),
        Rotation2d.fromDegrees(0.0));
    bluePOIs.reefScoring[7] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(180.0))),
        Rotation2d.fromDegrees(0.0));
    bluePOIs.reefScoring[8] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(240.0))),
        Rotation2d.fromDegrees(60.0));
    bluePOIs.reefScoring[9] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(240.0))),
        Rotation2d.fromDegrees(60.0));
    bluePOIs.reefScoring[10] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToLeftScoring.rotateBy(Rotation2d.fromDegrees(300.0))),
        Rotation2d.fromDegrees(120.0));
    bluePOIs.reefScoring[11] = new Pose2d(
        bluePOIs.reefCenter.plus(reefToRightScoring.rotateBy(Rotation2d.fromDegrees(300.0))),
        Rotation2d.fromDegrees(120.0));
  }

  public static final POIs redPOIs = new POIs();
  static {
    redPOIs.reefCenter = fieldCenter.plus(new Translation2d(4.284793, 0.0));
    var distanceBetweenReefs = redPOIs.reefCenter.getDistance(bluePOIs.reefCenter);
    for(int i = 0; i < bluePOIs.reefScoring.length; i++){
      redPOIs.reefScoring[i] = new Pose2d(bluePOIs.reefScoring[i].getX() + distanceBetweenReefs, bluePOIs.reefScoring[i].getY(), bluePOIs.reefScoring[i].getRotation());
    }
  }
}
