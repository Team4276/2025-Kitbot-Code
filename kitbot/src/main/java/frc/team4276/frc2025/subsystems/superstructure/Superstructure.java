package frc.team4276.frc2025.subsystems.superstructure;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase { //TODO: test logic


  @AutoLogOutput
  private boolean wantScore = false;

  public enum Goal {
    STOW,
    INTAKE,
    L1,
    L2,
    L3,
    CHARACTERIZING
  }

  private Goal desiredGoal = Goal.STOW;
  private Goal currentGoal = Goal.STOW;

  private Timer scoringTimer = new Timer();

  private double elevatorCharacterizationInput = 0.0;

  public Superstructure() {
    scoringTimer.restart();
  }

  @Override
  public void periodic() {

  }

  @AutoLogOutput
  public Goal getGoal() {
    return currentGoal;
  }

  public Command scoreCommand() {
    return Commands.runOnce(() -> wantScore = true);
  }

  public void acceptCharacterizationInput(double input){
    elevatorCharacterizationInput = input;
  }

  public double getFFCharacterizationVelocity(){
    return 0;
  }

  public void endCharacterizaton(){
  }
}
