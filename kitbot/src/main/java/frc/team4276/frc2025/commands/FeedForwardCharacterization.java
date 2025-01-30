package frc.team4276.frc2025.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FeedForwardCharacterization extends Command {
  private final double FF_START_DELAY = 2.0; // Secs
  private final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private final List<Double> velocitySamples = new LinkedList<>();
  private final List<Double> voltageSamples = new LinkedList<>();
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;

  private final Timer timer = new Timer();

  public FeedForwardCharacterization(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier) {
    addRequirements(subsystem);
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if (timer.get() < FF_START_DELAY) {
      voltageConsumer.accept(0.0);

    } else {
      double voltage = timer.get() * FF_RAMP_RATE;
      voltageConsumer.accept(voltage);
      velocitySamples.add(velocitySupplier.get());
      voltageSamples.add(voltage);
    }
  }

  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();

    int n = velocitySamples.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;
    for (int i = 0; i < n; i++) {
      sumX += velocitySamples.get(i);
      sumY += voltageSamples.get(i);
      sumXY += velocitySamples.get(i) * voltageSamples.get(i);
      sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
    }
    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

    NumberFormat formatter = new DecimalFormat("#0.00000");
    System.out.println("********** Drive FF Characterization Results **********");
    System.out.println("\tkS: " + formatter.format(kS));
    System.out.println("\tkV: " + formatter.format(kV));
  }
}
