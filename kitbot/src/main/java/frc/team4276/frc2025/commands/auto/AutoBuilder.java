package frc.team4276.frc2025.commands.auto;

import static frc.team4276.frc2025.commands.auto.AutoCommands.*;
import static frc.team4276.util.path.ChoreoUtil.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.Drive;

public class AutoBuilder {
  private final Drive drive;

  public AutoBuilder(Drive drive) {
    this.drive = drive;
  }

  public Command TestChoreoTraj(String name) {
    var traj1 = getChoreoSwerveTrajectory(name);

    return Commands.runOnce(
            () -> RobotState.getInstance().resetPose(traj1.getInitialPose(false).get()))
        .andThen(followTrajectory(drive, traj1));
  }

  public Command TestPPTraj(String name) {
    var traj1 = getPathPlannerTrajectoryFromChoreo(name);

    return Commands.runOnce(() -> RobotState.getInstance().resetPose(traj1.getInitialPose()))
        .andThen(followPathPlannerTrajectory(drive, traj1));
  }
}
