package frc.robot.subsystems.winch;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import java.util.function.DoubleSupplier;

public class WinchIOSparkMax implements WinchIO {
  // Hardware objects
  private final SparkMax spark;
  private Servo servo = null;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  // Private variables
  private double maxVoltage = 12.0;
  private double servoEngageValue = 0.0;
  private double servoReleaseValue = 0.0;

  public WinchIOSparkMax(
      int id,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double voltageLimit) {
    maxVoltage = voltageLimit;
    spark = new SparkMax(id, MotorType.kBrushless);

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage);
    sparkConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // Add a follower to the existing IO
  public WinchIOSparkMax addFollower(int id, boolean inverted) {
    SparkMax followerSpark = new SparkMax(id, MotorType.kBrushless);
    SparkMaxConfig followerSparkConfig = new SparkMaxConfig();
    followerSparkConfig.idleMode(IdleMode.kBrake).follow(spark, inverted);
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    return this;
  }

  // Add a servo controlled ratchet
  public WinchIOSparkMax addServo(int port, double engageValue, double releaseValue) {
    servo = new Servo(port);

    servoEngageValue = engageValue;
    servoReleaseValue = releaseValue;

    return this;
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {
    // Update inputs
    sparkStickyFault = false;
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getAppliedOutput, (value) -> inputs.appliedOutput = value);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.ratchetOutput = servo.get();
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    spark.setVoltage(voltage.in(Volts));
  }

  @Override
  public void engageRatchet() {
    if (servo != null) {
      servo.set(servoEngageValue);
    }
  }

  @Override
  public void releaseRatchet() {
    if (servo != null) {
      servo.set(servoReleaseValue);
    }
  }
}
