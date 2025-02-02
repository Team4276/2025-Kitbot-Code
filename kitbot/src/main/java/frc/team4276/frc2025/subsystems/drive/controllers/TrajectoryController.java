package frc.team4276.frc2025.subsystems.drive.controllers;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.RobotState;
import frc.team4276.util.LoggedTunableNumber;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {
  private final LoggedTunableNumber translationkP = new LoggedTunableNumber("TrajectoryController/Translation/kP", 4.0);
  private final LoggedTunableNumber translationkD = new LoggedTunableNumber("TrajectoryController/Translation/kD", 0.0);
  private final LoggedTunableNumber translationKTol = new LoggedTunableNumber(
      "TrajectoryController/Translation/Tolerance", 0.1);

  private final LoggedTunableNumber rotationkP = new LoggedTunableNumber("TrajectoryController/Rotation/kP", 3.0);
  private final LoggedTunableNumber rotationkD = new LoggedTunableNumber("TrajectoryController/Rotation/kD", 0.0);
  private final LoggedTunableNumber rotationKTol = new LoggedTunableNumber(
      "TrajectoryController/Rotation/ToleranceDegrees", 1.0);

  private final LoggedTunableNumber maxError = new LoggedTunableNumber("TrajectoryController/maxError", 0.75);

  private PathPlannerTrajectory traj;

  private double startTime = 0.0;
  private double timeOffset = 0.0;

  private boolean isFinished = true;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private final double[] moduleForces = { 0.0, 0.0, 0.0, 0.0 };
  private final double[] dummyForces = { 0.0, 0.0, 0.0, 0.0 };

  public TrajectoryController() {
    xController = new PIDController(translationkP.getAsDouble(), 0.0, translationkD.getAsDouble());
    yController = new PIDController(translationkP.getAsDouble(), 0.0, translationkD.getAsDouble());
    rotationController = new PIDController(rotationkP.getAsDouble(), 0.0, rotationkD.getAsDouble());
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(translationKTol.getAsDouble());
    yController.setTolerance(translationKTol.getAsDouble());
    rotationController.setTolerance(Math.toRadians(rotationKTol.getAsDouble()));
  }

  public void setTrajectory(PathPlannerTrajectory traj) {
    this.traj = traj;
    isFinished = false;
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  public ChassisSpeeds update(Pose2d currentPose) {    
    xController.setPID(translationkP.getAsDouble(), 0.0, translationkD.getAsDouble());
    xController.setTolerance(translationKTol.getAsDouble());

    yController.setPID(translationkP.getAsDouble(), 0.0, translationkD.getAsDouble());
    yController.setTolerance(translationKTol.getAsDouble());

    rotationController.setPID(rotationkP.getAsDouble(), 0.0, rotationkD.getAsDouble());
    rotationController.setTolerance(Math.toRadians(rotationKTol.getAsDouble()));

    var sampledState = traj.sample(getTrajectoryTime());

    if (sampledState.pose.getTranslation().getDistance(currentPose.getTranslation()) > maxError.getAsDouble()) {
      timeOffset += 0.02;

      var dummyState = traj.sample(getTrajectoryTime());

      sampledState = new PathPlannerTrajectoryState();
      sampledState.timeSeconds = dummyState.timeSeconds;
      sampledState.pose = dummyState.pose;
      sampledState.heading = dummyState.heading;
      sampledState.feedforwards = new DriveFeedforwards(dummyForces, dummyForces, dummyForces, dummyForces,
          dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(sampledState.pose);

    for (int i = 0; i < 4; i++) {
      moduleForces[i] = sampledState.feedforwards.linearForcesNewtons()[i];
    }

    double xError = sampledState.pose.getX() - currentPose.getTranslation().getX();
    double yError = sampledState.pose.getY() - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaError = MathUtil.angleModulus(sampledState.pose.getRotation().minus(currentPose.getRotation()).getRadians());
    double thetaFeedback = rotationController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        sampledState.fieldSpeeds.vxMetersPerSecond + xFeedback,
        sampledState.fieldSpeeds.vyMetersPerSecond + yFeedback,
        sampledState.fieldSpeeds.omegaRadiansPerSecond + thetaFeedback,
        currentPose.getRotation());

    Logger.recordOutput("Trajectory/SetpointPose", sampledState.pose);
    Logger.recordOutput("Trajectory/SetpointSpeeds/vx", sampledState.fieldSpeeds.vxMetersPerSecond);
    Logger.recordOutput("Trajectory/SetpointSpeeds/vy", sampledState.fieldSpeeds.vyMetersPerSecond);
    Logger.recordOutput("Trajectory/SetpointSpeeds/omega", sampledState.heading.getRadians());
    Logger.recordOutput("Trajectory/OutputSpeeds", outputSpeeds);
    Logger.recordOutput("Trajectory/TranslationError", Math.hypot(xError, yError));
    Logger.recordOutput("Trajectory/RotationError", thetaError);

    return outputSpeeds;
  }

  private double getTrajectoryTime() {
    return Timer.getTimestamp() - startTime - timeOffset;
  }

  public void stopTraj() {
    isFinished = true;
  }

  @AutoLogOutput(key = "Trajectory/Finished")
  public boolean isFinished() {
    return isFinished;
  }

  public double[] getModuleForces() {
    return moduleForces;
  }
}
