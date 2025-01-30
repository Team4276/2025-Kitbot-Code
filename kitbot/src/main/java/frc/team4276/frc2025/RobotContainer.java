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
package frc.team4276.frc2025;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.team4276.frc2025.Constants.RobotType;
import frc.team4276.frc2025.commands.FeedForwardCharacterization;
import frc.team4276.frc2025.commands.WheelRadiusCharacterization;
import frc.team4276.frc2025.commands.auto.AutoBuilder;
import frc.team4276.frc2025.subsystems.drive.Drive;
import frc.team4276.frc2025.subsystems.drive.GyroIO;
import frc.team4276.frc2025.subsystems.drive.GyroIOADIS;
import frc.team4276.frc2025.subsystems.drive.ModuleIO;
import frc.team4276.frc2025.subsystems.drive.ModuleIOSim;
import frc.team4276.frc2025.subsystems.drive.ModuleIOSpark;
import frc.team4276.frc2025.subsystems.roller.Roller;
import frc.team4276.frc2025.subsystems.roller.Roller.Goal;
import frc.team4276.frc2025.subsystems.roller.RollerIO;
import frc.team4276.frc2025.subsystems.roller.RollerIOSparkMax;
import frc.team4276.frc2025.subsystems.superstructure.Superstructure;
import frc.team4276.frc2025.subsystems.vision.Vision;
import frc.team4276.frc2025.subsystems.vision.VisionConstants;
import frc.team4276.frc2025.subsystems.vision.VisionIO;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVision;
import frc.team4276.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import frc.team4276.util.BetterXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems

    private Drive drive;
    private Superstructure superstructure;
    private Vision vision;

    @SuppressWarnings("unused")

    private AutoBuilder autoBuilder;
    private Roller roller;

    // Controller
    private final BetterXboxController driver = new BetterXboxController(0);
    private final CommandGenericHID keyboard0 = new CommandGenericHID(1);
    private final CommandGenericHID keyboard1 = new CommandGenericHID(2);
    private final CommandGenericHID keyboard2 = new CommandGenericHID(3);

    private final ScoringHelper scoringHelper = new ScoringHelper();

    // Dashboard inputs
    private final AutoSelector autoSelector = new AutoSelector();

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        if (Constants.getMode() != Constants.Mode.REPLAY) {
            switch (Constants.getMode()) {
                case REAL:
                    // Real robot, instantiate hardware IO implementations
                    drive = new Drive(
                            new GyroIOADIS(),
                            new ModuleIOSpark(0),
                            new ModuleIOSpark(1),
                            new ModuleIOSpark(2),
                            new ModuleIOSpark(3));

                    roller = new Roller(new RollerIOSparkMax(Ports.ALGAE_INTAKE_ROLLER, 20, true, false));
                              vision = new Vision(
              RobotState.getInstance()::addVisionMeasurement,
              new VisionIOPhotonVision(
                  VisionConstants.camera0Name, VisionConstants.robotToCamera0),
              new VisionIOPhotonVision(
                  VisionConstants.camera1Name, VisionConstants.robotToCamera1));
                    break;

                case SIM:
                    // Sim robot, instantiate physics sim IO implementations
                    drive = new Drive(
                            new GyroIO() {
                    },
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim(),
                            new ModuleIOSim());
                    roller = new Roller(new RollerIO() {
                    });
          vision = new Vision(
              RobotState.getInstance()::addVisionMeasurement,
              new VisionIOPhotonVisionSim(
                  VisionConstants.camera0Name,
                  VisionConstants.robotToCamera0,
                  RobotState.getInstance()::getEstimatedPose),
              new VisionIOPhotonVisionSim(
                  VisionConstants.camera1Name,
                  VisionConstants.robotToCamera1,
                  RobotState.getInstance()::getEstimatedPose));
                    break;

                default:
                    // Replayed robot, disable IO implementations
                    drive = new Drive(
                            new GyroIO() {
                    },
                            new ModuleIO() {
                    },
                            new ModuleIO() {
                    },
                            new ModuleIO() {
                    },
                            new ModuleIO() {
                    });
                    roller = new Roller(new RollerIO() {
                    });
                    vision = new Vision(
                        RobotState.getInstance()::addVisionMeasurement,
                        new VisionIO() {
                        },
                        new VisionIO() {
                        });
            }
        }

        autoBuilder = new AutoBuilder(drive);

        // Set up SysId routines
        autoSelector.addRoutine(
                "Drive Wheel Radius Characterization", new WheelRadiusCharacterization(drive));
        autoSelector.addRoutine(
                "Drive Simple FF Characterization",
                new FeedForwardCharacterization(
                        drive, drive::runCharacterization, drive::getFFCharacterizationVelocity));
        autoSelector.addRoutine(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoSelector.addRoutine(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoSelector.addRoutine(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoSelector.addRoutine(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();

        // Peace and quiet
        if (Constants.getType() == RobotType.SIMBOT) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
                drive.run(
                        () -> drive.feedTeleopInput(
                                driver.getLeftX(), -driver.getLeftY(), -driver.getRightX())));

        // Reset gyro to 0° when A button is pressed
        driver
                .a()
                .onTrue(
                        Commands.runOnce(
                                () -> RobotState.getInstance()
                                        .resetPose(
                                                new Pose2d(
                                                        RobotState.getInstance().getEstimatedPose().getTranslation(),
                                                        new Rotation2d())),
                                drive)
                                .ignoringDisable(true));

        keyboard0
                .button(1)
                .whileTrue(
                        Commands.startEnd(
                                () -> drive.setAutoAlignPosition(scoringHelper.getSelectedPose()),
                                drive::disableAutoAlign)
                                .until(drive::isAutoAligned));

        driver
                .rightBumper()
                .onTrue(
                        Commands.runOnce(() -> superstructure.scoreCommand()));

        driver
                .leftBumper()
                .whileTrue(
                        Commands.startEnd(
                                () -> drive.setHeadingGoal(
                                        () -> Rotation2d.fromDegrees(-90.0 + (ChoreoAllianceFlipUtil.shouldFlip() ? 180.0 : 0.0))),
                                drive::clearHeadingGoal));

        driver
                .leftTrigger()
                .whileTrue(roller.startEnd(() -> roller.setGoal(Goal.SCORE), () -> roller.setGoal(Goal.IDLE)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoBuilder.TestPPTraj("SimDemo");
        // new FeedForwardCharacterization(
        // drive, drive::runCharacterization, drive::getFFCharacterizationVelocity);
    }
}
