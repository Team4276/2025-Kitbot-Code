package frc.team4276.frc2025.subsystems.drive.controllers;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2025.RobotState;
import frc.team4276.frc2025.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrajectoryController {
  private Trajectory<SwerveSample> trajectory;
  private PathPlannerTrajectory ppTrajectory;
  private boolean isPPTraj = false;

  private double startTime = 0.0;
  private double timeOffset = 0.0;

  private boolean isFinished = true;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  // private Translation2d[] moduleForces = {
  //   new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()
  // };
  private final double[] moduleForces = {0.0, 0.0, 0.0, 0.0};
  private final double[] dummyForces = {0.0, 0.0, 0.0, 0.0};

  public TrajectoryController() {
    xController =
        new PIDController(DriveConstants.autoTranslationKp, 0.0, DriveConstants.autoTranslationKd);
    yController =
        new PIDController(DriveConstants.autoTranslationKp, 0.0, DriveConstants.autoTranslationKd);
    rotationController =
        new PIDController(DriveConstants.autoRotationKp, 0.0, DriveConstants.autoRotationKd);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(autoTranslationTol);
    yController.setTolerance(autoTranslationTol);
    rotationController.setTolerance(autoRotationTol);
  }

  public void setTrajectory(Trajectory<SwerveSample> traj) {
    trajectory = traj;
    isPPTraj = false;
    isFinished = false;
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  public void setPathPlannerTrajectory(PathPlannerTrajectory traj) {
    ppTrajectory = traj;
    isPPTraj = true;
    isFinished = false;
    startTime = Timer.getTimestamp();
    timeOffset = 0.0;
  }

  public ChassisSpeeds updatePP(Pose2d currentPose) {
    var sampledState = ppTrajectory.sample(getTrajectoryTime());

    if (sampledState.pose.getTranslation().getDistance(currentPose.getTranslation())
        > DriveConstants.autoMaxError) {
      timeOffset += 0.02;

      var dummyState = ppTrajectory.sample(getTrajectoryTime());

      sampledState = new PathPlannerTrajectoryState();
      sampledState.timeSeconds = dummyState.timeSeconds;
      sampledState.pose = dummyState.pose;
      sampledState.heading = dummyState.heading;
      sampledState.feedforwards =
          new DriveFeedforwards(dummyForces, dummyForces, dummyForces, dummyForces, dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(sampledState.pose);

    for (int i = 0; i < 4; i++) {
      moduleForces[i] = sampledState.feedforwards.linearForcesNewtons()[i];
    }

    double xError = sampledState.pose.getX() - currentPose.getTranslation().getX();
    double yError = sampledState.pose.getY() - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaError = sampledState.heading.minus(currentPose.getRotation()).getRadians();
    double thetaFeedback = rotationController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
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

  public ChassisSpeeds update(Pose2d currentPose) {
    if (isPPTraj) {
      return updatePP(currentPose);
    }
    var sampledState = trajectory.sampleAt(getTrajectoryTime(), false);

    // Auto finished or interrupted
    if (sampledState.isEmpty() || getTrajectoryTime() > trajectory.getTotalTime()) {
      isFinished = true;
      return new ChassisSpeeds();
    }

    SwerveSample targetState = sampledState.get();
    currentPose = targetState.getPose();

    if (targetState.getPose().getTranslation().getDistance(currentPose.getTranslation())
        > DriveConstants.autoMaxError) {
      timeOffset += 0.02;

      var dummyState = trajectory.sampleAt(getTrajectoryTime(), false);
      if (dummyState.isEmpty()) {
        isFinished = true;
        return new ChassisSpeeds();
      }

      targetState =
          new SwerveSample(
              dummyState.get().t,
              dummyState.get().x,
              dummyState.get().y,
              dummyState.get().heading,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              dummyForces,
              dummyForces);
    }

    RobotState.getInstance().setTrajectorySetpoint(targetState.getPose());

    for (int i = 0; i < 4; i++) {
      // moduleForces[i] =
      //     new Translation2d(targetState.moduleForcesX()[i], targetState.moduleForcesY()[i]);
    }

    double xError = targetState.x - currentPose.getTranslation().getX();
    double yError = targetState.y - currentPose.getTranslation().getY();
    double xFeedback = xController.calculate(0.0, xError);
    double yFeedback = yController.calculate(0.0, yError);
    double thetaFF = targetState.omega;
    double thetaError =
        Rotation2d.fromRadians(targetState.heading).minus(currentPose.getRotation()).getRadians();
    double thetaFeedback = rotationController.calculate(0.0, thetaError);

    ChassisSpeeds outputSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetState.vx + xFeedback,
            targetState.vy + yFeedback,
            thetaFF + thetaFeedback,
            currentPose.getRotation());

    Logger.recordOutput("Trajectory/SetpointPose", targetState.getPose());
    Logger.recordOutput("Trajectory/SetpointSpeeds/vx", targetState.vx);
    Logger.recordOutput("Trajectory/SetpointSpeeds/vy", targetState.vy);
    Logger.recordOutput("Trajectory/SetpointSpeeds/omega", targetState.omega);
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
