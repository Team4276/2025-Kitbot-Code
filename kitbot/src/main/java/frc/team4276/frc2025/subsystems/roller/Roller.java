package frc.team4276.frc2025.subsystems.roller;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {

    public enum Goal {
        IDLE(() -> 0.0),
        INTAKE(() -> 12.0),
        HOLD(() -> 2.0),
        SCORE(() -> -12.0);

        private final DoubleSupplier voltageGoal;

        private Goal(DoubleSupplier voltageGoal) {
            this.voltageGoal = voltageGoal;
        }

        private double getVolts() {
            return voltageGoal.getAsDouble();
        }

    }

    private Goal goal = Goal.IDLE;
    private boolean hasGamePiece = false;

    private final RollerIO io;
    private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

    public Roller(RollerIO io) {
        this.io = io;

        setDefaultCommand(setGoalCommand(Goal.IDLE));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Roller", inputs);

        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE;
        }

        if (inputs.supplyCurrentAmps > 40.0) {
            hasGamePiece = true;
        }

        if (goal == Goal.SCORE) {
            hasGamePiece = false;
        }

        io.runVolts(goal.getVolts());
        Logger.recordOutput("Roller/Goal", goal);
    }

    @AutoLogOutput
    public Goal getGoal() {
        return goal;
    }

    @AutoLogOutput
    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Command setGoalCommand(Goal goal) {
        return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.IDLE), this);
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }
}
