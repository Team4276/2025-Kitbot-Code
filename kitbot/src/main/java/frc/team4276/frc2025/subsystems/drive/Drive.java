// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.team4276.frc2025.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team4276.frc2025.Constants;
import frc.team4276.frc2025.Constants.Mode;
import frc.team4276.frc2025.RobotState;
import static frc.team4276.frc2025.subsystems.drive.DriveConstants.driveConfig;
import static frc.team4276.frc2025.subsystems.drive.DriveConstants.kinematics;
import static frc.team4276.frc2025.subsystems.drive.DriveConstants.maxSteerVelocity;
import static frc.team4276.frc2025.subsystems.drive.DriveConstants.wheelRadiusMeters;
import frc.team4276.frc2025.subsystems.drive.controllers.AutoAlignController;
import frc.team4276.frc2025.subsystems.drive.controllers.HeadingController;
import frc.team4276.frc2025.subsystems.drive.controllers.TeleopDriveController;
import frc.team4276.frc2025.subsystems.drive.controllers.TrajectoryController;
import frc.team4276.util.swerve.SwerveSetpointGenerator;

public class Drive extends SubsystemBase {
  public enum DriveMode {
    /** Driving with input from driver joysticks. (Default) */
    TELEOP,

    /** Driving based on a trajectory. */
    TRAJECTORY,

    /** Driving to a location on the field automatically. */
    AUTO_ALIGN,

    /**
     * Characterizing (modules oriented forwards, motor outputs supplied
     * externally).
     */
    CHARACTERIZATION,

    /** Running wheel radius characterization routine (spinning in circle) */
    WHEEL_RADIUS_CHARACTERIZATION
  }

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
      AlertType.kError);

  private SwerveModulePosition[] lastModulePositions = null;
  private double lastTime = 0.0;

  private boolean useSetpointGenerator = false;
  private final SwerveSetpointGenerator swerveSetpointGenerator = new SwerveSetpointGenerator(driveConfig,
      maxSteerVelocity);
  private SwerveSetpoint prevSetpoint;

  private DriveMode mode = DriveMode.TELEOP;
  private boolean isHeadingControlled = false;

  private final TeleopDriveController teleopDriveController = new TeleopDriveController();
  private final HeadingController headingController = new HeadingController();
  private final TrajectoryController trajectoryController = new TrajectoryController();
  private final AutoAlignController autoAlignController = new AutoAlignController();

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private double characterizationInput = 0.0;

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    prevSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      // Stop moving when disabled
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
      }

      boolean includeMeasurement = true;
      if (lastModulePositions != null) {
        double dt = sampleTimestamps[i] - lastTime;
        for (int j = 0; j < modules.length; j++) {
          double velocity = (modulePositions[j].distanceMeters - lastModulePositions[j].distanceMeters) / dt;
          double omega = modulePositions[j].angle.minus(lastModulePositions[j].angle).getRadians() / dt;
          // Check if delta is too large
          if (Math.abs(omega) > DriveConstants.maxSpeed * 5.0
              || Math.abs(velocity) > DriveConstants.maxAngularSpeed * 5.0) {
            includeMeasurement = false;
            break;
          }
        }
      }
      // If delta isn't too large we can include the measurement.
      if (includeMeasurement) {
        lastModulePositions = modulePositions;
        RobotState.getInstance()
            .addOdometryObservation(
                sampleTimestamps[i],
                gyroInputs.connected ? gyroInputs.yawPosition : null,
                modulePositions);
        lastTime = sampleTimestamps[i];
      }
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.getMode() != Mode.SIM);

    Pose2d currentPose = RobotState.getInstance().getEstimatedVisionPose();
    currentPose = RobotState.getInstance().getEstimatedPose();

    switch (mode) {
      case TELEOP:
        desiredSpeeds = teleopDriveController.update(currentPose.getRotation());

        if (isHeadingControlled) {
          desiredSpeeds.omegaRadiansPerSecond = headingController.update(currentPose.getRotation().getRadians());
        }

        break;
      case AUTO_ALIGN:
        desiredSpeeds = autoAlignController.update(currentPose);

        break;
      case TRAJECTORY:
        desiredSpeeds = trajectoryController.update(currentPose);

        if (isHeadingControlled) {
          desiredSpeeds.omegaRadiansPerSecond = headingController.update(currentPose.getRotation().getRadians());
        }

        break;
      case CHARACTERIZATION:
        for (Module module : modules) {
          module.runCharacterization(characterizationInput);
        }

        break;
      case WHEEL_RADIUS_CHARACTERIZATION:
        desiredSpeeds = new ChassisSpeeds(0.0, 0.0, characterizationInput);

        break;
      default:
        break;
    }

    if (mode != DriveMode.CHARACTERIZATION) {
      // Calculate setpoints
      ChassisSpeeds setpointSpeeds;
      SwerveModuleState[] setpointStates;
      if (useSetpointGenerator) {
        prevSetpoint = swerveSetpointGenerator.generateSetpoint(prevSetpoint, desiredSpeeds, 0.02);
        setpointSpeeds = prevSetpoint.robotRelativeSpeeds();
        setpointStates = prevSetpoint.moduleStates();
      } else {
        setpointSpeeds = ChassisSpeeds.discretize(desiredSpeeds, 0.02);
        setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
      }

      SwerveModuleState[] setpointTorques = new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
      };

      // Send setpoints to modules
      for (int i = 0; i < 4; i++) {
        if (mode == DriveMode.TRAJECTORY) {
          // setpointTorques[i] =
          // new SwerveModuleState(
          // trajectoryController
          // .getModuleForces()[i]
          // .getAngle()
          // .minus(setpointStates[i].angle)
          // .getCos()
          // * trajectoryController.getModuleForces()[i].getNorm()
          // * wheelRadiusMeters,
          // setpointStates[i].angle);
          setpointTorques[i] = new SwerveModuleState(
              trajectoryController.getModuleForces()[i] * wheelRadiusMeters,
              setpointStates[i].angle);
        } else {
          setpointTorques[i] = new SwerveModuleState(0.0, setpointStates[i].angle);
        }
        modules[i].runSetpoint(setpointStates[i], setpointTorques[i]);
      }

      // Log optimized setpoints (runSetpoint mutates each state)
      Logger.recordOutput("Drive/SetpointSpeeds", setpointSpeeds);
      Logger.recordOutput("Drive/SwerveStates/OptimizedSetpoints", setpointStates);
      Logger.recordOutput("Drive/SwerveStates/Torques", setpointTorques);
    }

    Logger.recordOutput(
        "Drive/SwerveStates/Setpoints",
        kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(desiredSpeeds, 0.02)));
    Logger.recordOutput("Drive/DesiredSpeeds", desiredSpeeds);
    Logger.recordOutput("Drive/DriveMode", mode);
  }

  public void feedTeleopInput(double x, double y, double omega) {
    teleopDriveController.feedDriveInput(x, y, omega);
    autoAlignController.feedTeleopInput(x, y, omega);
  }

  public void setHeadingGoal(Supplier<Rotation2d> goalHeadingSupplier) {
    headingController.setTarget(goalHeadingSupplier);
    isHeadingControlled = true;
  }

  public void clearHeadingGoal() {
    isHeadingControlled = false;
  }

  public void setTrajectory(Trajectory<SwerveSample> traj) {
    if (DriverStation.isAutonomousEnabled()) {
      trajectoryController.setTrajectory(traj);
      mode = DriveMode.TRAJECTORY;
    }
  }

  public void setPathPlannerTrajectory(PathPlannerTrajectory traj) {
    if (DriverStation.isAutonomousEnabled()) {
      trajectoryController.setPathPlannerTrajectory(traj);
      mode = DriveMode.TRAJECTORY;
    }
  }

  public boolean isTrajectoryCompleted() {
    return trajectoryController.isFinished();
  }

  public void clearTrajectory() {
    trajectoryController.stopTraj();
    mode = DriveMode.TELEOP;
  }

  public void setAutoAlignPosition(Pose2d pose) {
    autoAlignController.setSetpoint(pose);
    mode = DriveMode.AUTO_ALIGN;
  }

  public boolean isAutoAligned() {
    return autoAlignController.atGoal();
  }

  public void disableAutoAlign() {
    mode = DriveMode.TELEOP;
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    characterizationInput = output;
    mode = DriveMode.CHARACTERIZATION;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  public void endCharacterizaton() {
    mode = DriveMode.TELEOP;
  }

  public void runWheelRadiusCharacterization(double omegaSpeed) {
    characterizationInput = omegaSpeed;
    mode = DriveMode.WHEEL_RADIUS_CHARACTERIZATION;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
}
