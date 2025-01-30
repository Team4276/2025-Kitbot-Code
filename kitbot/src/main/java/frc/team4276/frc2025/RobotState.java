package frc.team4276.frc2025;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2025.field.FieldConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  private SwerveModulePosition[] lastWheelPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private Rotation2d lastGyroAngle = new Rotation2d();

  private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, lastGyroAngle,
      lastWheelPositions, new Pose2d());
  private SwerveDrivePoseEstimator poseEstimatorVision = new SwerveDrivePoseEstimator(kinematics, lastGyroAngle,
      lastWheelPositions, new Pose2d());

  private Pose2d trajectorySetpoint = new Pose2d();

  private FieldConstants.POIs POIs = FieldConstants.bluePOIs;

  private static RobotState mInstance;

  private boolean enableSimTrajPoseEstimation = true;

  public static RobotState getInstance() {
    if (mInstance == null) {
      mInstance = new RobotState();
    }
    return mInstance;
  }

  private RobotState() {
  }

  public FieldConstants.POIs getPOIs() {
    return POIs;
  }

  public void setBlue() {
    POIs = FieldConstants.bluePOIs;
  }

  public void setRed() {
    POIs = FieldConstants.redPOIs;
  }

  /** Resets the current odometry pose. */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
    poseEstimatorVision.resetPose(pose);
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
    trajectorySetpoint = setpoint;
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d yaw, SwerveModulePosition[] wheelPositions) {
    // Update gyro angle
    if (yaw == null) {
      // Derive from kinematics
      yaw = lastGyroAngle.rotateBy(
          new Rotation2d(kinematics.toTwist2d(wheelPositions, lastWheelPositions).dtheta));
      lastGyroAngle = yaw;
    }

    lastWheelPositions = wheelPositions;

    poseEstimator.updateWithTime(timestamp, yaw, wheelPositions);
    poseEstimatorVision.updateWithTime(timestamp, yaw, wheelPositions);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimatorVision.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    // Temp until i get the sim to be consistent
    return useTrajectorySetpoint() ? trajectorySetpoint : poseEstimator.getEstimatedPosition();
  }

  @AutoLogOutput(key = "RobotState/EstimatedVisionPose")
  public Pose2d getEstimatedVisionPose(){
    return poseEstimatorVision.getEstimatedPosition();
  }

  private boolean useTrajectorySetpoint() {
    return enableSimTrajPoseEstimation
        ? false
        : Constants.getMode() == Constants.Mode.SIM && DriverStation.isAutonomousEnabled();
  }

  public Pose2d getTrajectorySetpoint() {
    return trajectorySetpoint;
  }
}
