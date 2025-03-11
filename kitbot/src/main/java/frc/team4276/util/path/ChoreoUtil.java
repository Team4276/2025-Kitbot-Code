package frc.team4276.util.path;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4276.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class ChoreoUtil {
  private ChoreoUtil() {}

  // Assume swerve sample and if not then we screwed ig
  /** Loads and flips trajectory accordingly */
  public static Trajectory<SwerveSample> getChoreoTrajectory(String name) {
    return getChoreoTrajectory(name, false);
  }

  public static Trajectory<SwerveSample> getChoreoTrajectory(
      String name, boolean mirrorLengthwise) {
    Optional<Trajectory<SwerveSample>> uncheckedTraj = Choreo.loadTrajectory(name);
    try {
      var traj = uncheckedTraj.orElseThrow();

      if (mirrorLengthwise) {
        traj = mirrorLengthwise(traj);
      }

      return AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj;
    } catch (Exception e) {
      System.out.println("Failed to load trajectory " + name);
      return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
    }
  }

  // Assume swerve sample and if not then we screwed ig
  /** Loads and flips trajectory accordingly */
  public static Trajectory<SwerveSample> getChoreoTrajectory(String name, int split) {
    return getChoreoTrajectory(name, false, split);
  }

  public static Trajectory<SwerveSample> getChoreoTrajectory(
      String name, boolean mirrorLengthwise, int split) {
    Optional<Trajectory<SwerveSample>> uncheckedTraj = Choreo.loadTrajectory(name);
    try {
      var traj = uncheckedTraj.orElseThrow().getSplit(split).orElseThrow();

      if (mirrorLengthwise) {
        traj = mirrorLengthwise(traj);
      }

      return AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj;
    } catch (Exception e) {
      System.out.println("Failed to load split " + split + " of trajectory " + name);
      return new Trajectory<SwerveSample>(name, List.of(), List.of(), List.of());
    }
  }

  public static SwerveSample mirrorLengthwise(SwerveSample sample) {
    Pose2d pose = PathUtil.mirrorLengthwise(sample.getPose());
    ChassisSpeeds speeds = PathUtil.mirrorLengthwise(sample.getChassisSpeeds());

    return new SwerveSample(
        sample.t,
        pose.getX(),
        pose.getY(),
        pose.getRotation().getRadians(),
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        sample.ax,
        -sample.ay,
        -sample.alpha,
        sample.moduleForcesX(),
        Arrays.stream(sample.moduleForcesY()).map(y -> -y).toArray());
  }

  public static Trajectory<SwerveSample> mirrorLengthwise(Trajectory<SwerveSample> traj) {
    List<SwerveSample> mirrored = new ArrayList<SwerveSample>();
    for (var sample : traj.samples()) {
      mirrored.add(mirrorLengthwise(sample));
    }

    return new Trajectory<SwerveSample>(traj.name(), mirrored, traj.splits(), traj.events());
  }
}
