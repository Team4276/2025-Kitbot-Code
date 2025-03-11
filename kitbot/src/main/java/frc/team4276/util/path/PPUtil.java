package frc.team4276.util.path;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import frc.team4276.util.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.List;

public class PPUtil {
  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(String name) {
    return getPathPlannerTrajectoryFromChoreo(name, false);
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(
      String name, boolean mirrorLengthwise) {
    try {
      var traj =
          PathPlannerPath.fromChoreoTrajectory(name)
              .generateTrajectory(
                  new ChassisSpeeds(), Rotation2d.kZero, DriveConstants.driveConfig);

      if (mirrorLengthwise) {
        traj = mirrorLengthwise(traj);
      }

      return AllianceFlipUtil.shouldFlip() ? traj.flip() : traj;
    } catch (Exception e) {
      System.out.println("Failed to load Choreo Trajectory from PPlib " + name);
      System.out.println(e);
      return new PathPlannerTrajectory(List.of(new PathPlannerTrajectoryState()));
    }
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(String name, int split) {
    return getPathPlannerTrajectoryFromChoreo(name, false, split);
  }

  public static PathPlannerTrajectory getPathPlannerTrajectoryFromChoreo(
      String name, boolean mirrorLengthwise, int split) {
    try {
      var traj =
          PathPlannerPath.fromChoreoTrajectory(name, split)
              .generateTrajectory(
                  new ChassisSpeeds(), Rotation2d.kZero, DriveConstants.driveConfig);

      if (mirrorLengthwise) {
        traj = mirrorLengthwise(traj);
      }

      return AllianceFlipUtil.shouldFlip() ? traj.flip() : traj;
    } catch (Exception e) {
      System.out.println(
          "Failed to load split " + split + " of Choreo Trajectory from PPlib " + name);
      System.out.println(e);
      return new PathPlannerTrajectory(List.of(new PathPlannerTrajectoryState()));
    }
  }

  public static PathPlannerTrajectory mirrorLengthwise(PathPlannerTrajectory trajectory) {
    List<PathPlannerTrajectoryState> mirroredStates = new ArrayList<>();
    for (var state : trajectory.getStates()) {
      mirroredStates.add(mirrorLengthwise(state));
    }
    return new PathPlannerTrajectory(mirroredStates, trajectory.getEvents());
  }

  private static final double[] dummyList = {0.0, 0.0, 0.0, 0.0};

  public static PathPlannerTrajectoryState mirrorLengthwise(PathPlannerTrajectoryState state) {
    var flipped = new PathPlannerTrajectoryState();

    flipped.timeSeconds = state.timeSeconds;
    flipped.linearVelocity = state.linearVelocity;
    flipped.pose = PathUtil.mirrorLengthwise(state.pose);
    flipped.feedforwards =
        new DriveFeedforwards(dummyList, dummyList, dummyList, dummyList, dummyList);
    flipped.fieldSpeeds = PathUtil.mirrorLengthwise(state.fieldSpeeds);
    flipped.heading = PathUtil.mirrorLengthwise(state.heading);

    return flipped;
  }
}
