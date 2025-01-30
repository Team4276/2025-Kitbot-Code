package frc.team4276.frc2025.commands.auto;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAllianceFlipUtil;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.field.FieldConstants;
import frc.team4276.frc2025.subsystems.drive.Drive;
import java.util.function.Supplier;

public class AutoCommands {
  private AutoCommands() {}

  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(() -> RobotState.getInstance().resetPose(pose));
  }

  /** Creates a command that follows a trajectory, command ends when the trajectory is finished */
  public static Command followTrajectory(Drive drive, Trajectory<SwerveSample> trajectory) {
    return followTrajectory(drive, () -> trajectory);
  }

  /** Creates a command that follows a trajectory, command ends when the trajectory is finished */
  public static Command followTrajectory(
      Drive drive, Supplier<Trajectory<SwerveSample>> trajectorySupplier) {
    return Commands.startEnd(
            () -> drive.setTrajectory(trajectorySupplier.get()), drive::clearTrajectory)
        .until(drive::isTrajectoryCompleted);
  }

  /** Creates a command that follows a trajectory, command ends when the trajectory is finished */
  public static Command followPathPlannerTrajectory(Drive drive, PathPlannerTrajectory trajectory) {
    return followPathPlannerTrajectory(drive, () -> trajectory);
  }

  /** Creates a command that follows a trajectory, command ends when the trajectory is finished */
  public static Command followPathPlannerTrajectory(
      Drive drive, Supplier<PathPlannerTrajectory> trajectorySupplier) {
    return Commands.startEnd(
            () -> drive.setPathPlannerTrajectory(trajectorySupplier.get()), drive::clearTrajectory)
        .until(drive::isTrajectoryCompleted);
  }

  /**
   * Returns whether robot has crossed x boundary, accounting for alliance flip
   *
   * @param xPosition X position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed x coordinate towards center line or away
   *     from center line
   */
  public static boolean xCrossed(double xPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getTrajectorySetpoint();
    if (ChoreoAllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  /** Command that waits for x boundary to be crossed. See {@link #xCrossed(double, boolean)} */
  public static Command waitUntilXCrossed(double xPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenterline));
  }

  /**
   * Returns whether robot has crossed y boundary, accounting for alliance flip
   *
   * @param yPosition Y position coordinate on blue side of field.
   * @param towardsCenterline Whether to wait until passed y coordinate towards center line or away
   *     from center line
   */
  public static boolean yCrossed(double yPosition, boolean towardsCenterline) {
    Pose2d robotPose = RobotState.getInstance().getTrajectorySetpoint();
    if (ChoreoAllianceFlipUtil.shouldFlip()) {
      if (towardsCenterline) {
        return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
      } else {
        return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
      }
    } else {
      if (towardsCenterline) {
        return robotPose.getY() > yPosition;
      } else {
        return robotPose.getY() < yPosition;
      }
    }
  }

  /** Command that waits for y boundary to be crossed. See {@link #yCrossed(double, boolean)} */
  public static Command waitUntilYCrossed(double yPosition, boolean towardsCenterline) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsCenterline));
  }
}
