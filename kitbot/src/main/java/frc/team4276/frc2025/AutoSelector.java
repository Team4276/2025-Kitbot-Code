package frc.team4276.frc2025;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team4276.util.VirtualSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector extends VirtualSubsystem {
  private static final AutoRoutine defaultRoutine = new AutoRoutine("Do Nothing", Commands.none());

  private final LoggedDashboardChooser<AutoRoutine> routineChooser;

  private AutoRoutine lastRoutine;

  public AutoSelector() {
    routineChooser = new LoggedDashboardChooser<>("Auto/Routine");
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;
  }

  /** Registers a new auto routine that can be selected. */
  public void addRoutine(String name, Command command) {
    routineChooser.addOption(name, new AutoRoutine(name, command));
  }

  /** Returns the selected auto command. */
  public Command getCommand() {
    return lastRoutine.command();
  }

  /** Returns the name of the selected routine. */
  public String getSelectedName() {
    return lastRoutine.name();
  }

  public void periodic() {
    // Skip updates when actively running in auto
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null) {
      return;
    }

    // Update the list of questions
    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }

    // Update the routine and responses
    lastRoutine = selectedRoutine;
  }

  /** A customizable auto routine associated with a single command. */
  private static final record AutoRoutine(String name, Command command) {}
}
