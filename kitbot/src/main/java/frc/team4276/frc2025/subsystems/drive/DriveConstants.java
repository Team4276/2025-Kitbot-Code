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

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.team4276.frc2025.Ports;

public class DriveConstants {
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(23.5);
  public static final double wheelBase = Units.inchesToMeters(23.5);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  public static final double maxSpeed = 4.8;
  public static final double maxAccel = 8.9;
  public static final double maxAngularSpeed = maxSpeed / driveBaseRadius;
  public static final double maxAngularAccel = 20.0;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-1.539);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-0.105);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(2.226);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-2.845);

  // Device CAN IDs
  public static final int frontLeftDriveCanId = Ports.FRONT_LEFT_DRIVE;
  public static final int frontRightDriveCanId = Ports.FRONT_RIGHT_DRIVE;
  public static final int backLeftDriveCanId = Ports.BACK_LEFT_DRIVE;
  public static final int backRightDriveCanId = Ports.BACK_RIGHT_DRIVE;

  public static final int frontLeftTurnCanId = Ports.FRONT_LEFT_TURN;
  public static final int frontRightTurnCanId = Ports.FRONT_RIGHT_TURN;
  public static final int backLeftTurnCanId = Ports.BACK_LEFT_TURN;
  public static final int backRightTurnCanId = Ports.BACK_RIGHT_TURN;

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
  public static final double drivingMotorPinionTeeth = 13.0;
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (drivingMotorPinionTeeth * 15.0); // MAXSwerve with
  // x pinion teeth
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);
  public static final double maxSteerVelocity =
      driveGearbox.freeSpeedRadPerSec / driveMotorReduction;

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.001524;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.0;
  public static final double driveKv =
      12.0 / (driveGearbox.freeSpeedRadPerSec / driveMotorReduction);
  public static final double driveSimP = 1.0;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.028;
  public static final double driveSimKv = 0.1;

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = 9424.0 / 203.0;
  public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 1.0;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  public static final double snapKp = 0.5;
  public static final double snapKi = 0.0;
  public static final double snapKd = 0.1;
  public static final double snapPositionTolerance = 1.0;

  public static final double autoTranslationKp = 4.0;
  public static final double autoTranslationKd = 0.0;
  public static final double autoTranslationTol = 0.1;
  public static final double autoRotationKp = 0.0;
  public static final double autoRotationKd = 0.0;
  public static final double autoRotationTol = Math.toRadians(1.0);
  public static final double autoMaxError = 0.75; // Meters

  public static final double autoAlignTranslationKp = 1.0;
  public static final double autoAlignTranslationKd = 0.0;
  public static final double autoAlignTranslationTol = 0.1;
  public static final double autoAlignRotationKp = 1.0;
  public static final double autoAlignRotationKd = 0.0;
  public static final double autoAlignRotationTol = Math.toRadians(1.0);

  // PathPlanner configuration
  public static final double robotMassKg = 56.699;
  public static final double robotMOI = 5.267513460399;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig driveConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeed,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final double ffkT = 1.0 / DCMotor.getNEO(1).KtNMPerAmp;
}
