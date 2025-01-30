package frc.team4276.frc2025.subsystems.roller;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import static frc.team4276.util.SparkUtil.ifOk;
import static frc.team4276.util.SparkUtil.sparkStickyFault;
import static frc.team4276.util.SparkUtil.tryUntilOk;

public class RollerIOSparkMax implements RollerIO {

    private final SparkMax motor;
    private final Debouncer motorConnectedDebounce = new Debouncer(0.5);

    public RollerIOSparkMax(int id, int currentLimit, boolean invert, boolean brake) {
        motor = new SparkMax(id, MotorType.kBrushed);

        var config = new SparkMaxConfig();
        config
                .smartCurrentLimit(currentLimit)
                .inverted(invert)
                .idleMode(brake ? IdleMode.kBrake : IdleMode.kBrake);
        config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
        tryUntilOk(
                motor,
                5,
                ()
                -> motor.configure(
                        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        sparkStickyFault = false;
        ifOk(
                motor,
                new DoubleSupplier[]{motor::getAppliedOutput, motor::getBusVoltage},
                (values) -> inputs.appliedVoltage = values[0] * values[1]);
        ifOk(motor, motor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
        ifOk(motor, motor::getMotorTemperature, (value) -> inputs.tempCelsius = value);
        inputs.connected = motorConnectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
