package frc.team4276.frc2025.subsystems.drive.controllers;

import static frc.team4276.frc2025.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController {
  private final ProfiledPIDController translationController;
  private final ProfiledPIDController headingController;
  private final Timer toleranceTimer = new Timer();
  private final double toleranceTime = 0.5;

  @AutoLogOutput(key = "AutoAlign/SetpointPose")
  private Pose2d setpoint = new Pose2d();

  private TeleopDriveController teleopDriveController = new TeleopDriveController();

  private boolean cancelX = false;
  private boolean cancelY = false;
  private boolean cancelTheta = false;

  public AutoAlignController() {
    translationController = new ProfiledPIDController(
        autoAlignTranslationKp,
        0,
        autoAlignTranslationKd,
        new TrapezoidProfile.Constraints(maxSpeed, maxAccel));
    headingController = new ProfiledPIDController(
        autoAlignRotationKp,
        0,
        autoAlignRotationKd,
        new TrapezoidProfile.Constraints(maxAngularSpeed, maxAngularAccel));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    translationController.setTolerance(autoAlignTranslationTol);
    headingController.setTolerance(autoAlignRotationTol);

    toleranceTimer.restart();
  }

  public void setSetpoint(Pose2d pose) {
    this.setpoint = pose;
    toleranceTimer.reset();
    cancelX = false;
    cancelY = false;
    cancelTheta = false;
  }

  public void feedTeleopInput(double x, double y, double omega){
    teleopDriveController.feedDriveInput(x, y, omega);
  }

  private ChassisSpeeds updateContoller(Pose2d currentPose) {
    Translation2d trans = currentPose.getTranslation().minus(setpoint.getTranslation());
    Translation2d linearOutput = new Translation2d();
    if (trans.getNorm() > 1e-6) {
      linearOutput = new Translation2d(
          translationController.calculate(trans.getNorm(), 0.0), trans.getAngle());
    }

    double thetaError = setpoint.getRotation().minus(currentPose.getRotation()).getRadians();
    double omega = headingController.calculate(thetaError, 0.0);

    if (!translationController.atGoal() || !headingController.atGoal()) {
      toleranceTimer.reset();
    }

    Logger.recordOutput("AutoAlign/DistanceMeasured", trans.getNorm());
    Logger.recordOutput("AutoAlign/DistanceSetpoint", translationController.getSetpoint().position);
    Logger.recordOutput("AutoAlign/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("AutoAlign/ThetaSetpoint", headingController.getSetpoint().position);
    Logger.recordOutput(
        "AutoAlign/StateSetpoint",
        new Pose2d(linearOutput, new Rotation2d(headingController.getSetpoint().position)));

    return new ChassisSpeeds(linearOutput.getX(), linearOutput.getY(), omega);

  }

  @AutoLogOutput(key = "AutoAlign/Output")
  public ChassisSpeeds update(Pose2d currentPose) {
    ChassisSpeeds controllerSpeeds = updateContoller(currentPose);
    
    var teleopSpeeds = teleopDriveController.updateRaw(currentPose.getRotation());

    cancelX = cancelX || Math.abs(teleopSpeeds.vxMetersPerSecond) > 1e-6;
    cancelY = cancelY || Math.abs(teleopSpeeds.vyMetersPerSecond) > 1e-6;
    cancelTheta = cancelTheta || Math.abs(teleopSpeeds.omegaRadiansPerSecond) > 1e-6;
    
    Logger.recordOutput("AutoAlign/CancelX", cancelX);
    Logger.recordOutput("AutoAlign/CancelY", cancelY);
    Logger.recordOutput("AutoAlign/CancelTheta", cancelTheta);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
      cancelX ? teleopSpeeds.vxMetersPerSecond : controllerSpeeds.vxMetersPerSecond, 
      cancelY ? teleopSpeeds.vyMetersPerSecond : controllerSpeeds.vyMetersPerSecond,
      cancelTheta ? teleopSpeeds.omegaRadiansPerSecond : controllerSpeeds.omegaRadiansPerSecond,
      currentPose.getRotation());
  }

  @AutoLogOutput(key = "AutoAlign/AtGoal")
  public boolean atGoal() {
    return toleranceTimer.hasElapsed(toleranceTime);
  }
}
