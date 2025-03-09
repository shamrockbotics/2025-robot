package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.function.DoubleSupplier;

public class ElevatorIOSparkRIO implements ElevatorIO {
  private final double zeroOffsetRads;

  // Hardware objects
  private final SparkBase spark;
  private final DutyCycleEncoder encoder;

  // Closed loop controllers
  private final PIDController controller;
  private double setpoint = 0.0;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public ElevatorIOSparkRIO(
      int id1,
      int id2,
      int encoderPort,
      double zeroOffsetRads,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double turnKp,
      double turnKd) {
    this(
        id1,
        encoderPort,
        zeroOffsetRads,
        motorInverted,
        encoderInverted,
        encoderPositionFactor,
        encoderVelocityFactor,
        currentLimit,
        turnKp,
        turnKd);
    SparkBase followerSpark = new SparkMax(id2, MotorType.kBrushless);
    SparkMaxConfig followerSparkConfig = new SparkMaxConfig();
    followerSparkConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage)
        .follow(spark, true);
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    encoder.setInverted(encoderInverted);
  }

  public ElevatorIOSparkRIO(
      int id,
      int encoderPort,
      double zeroOffsetRads,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double kp,
      double kd) {
    this.zeroOffsetRads = zeroOffsetRads;
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(encoderPort);
    controller = new PIDController(kp, 0.0, kd);

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage);
    sparkConfig
        .absoluteEncoder
        .inverted(encoderInverted)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .averageDepth(2);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update inputs
    sparkStickyFault = false;
    inputs.currentAngleRads = getAngle();
    inputs.currentHeight = getAngle() * .049;
    inputs.targetAngleRads = setpoint;
    inputs.velocityRadsPerSec = 0.0; // figure out velocity later
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault && encoder.isConnected());
  }

  @Override
  public void setPosition(double angleRads) {
    setpoint = angleRads;
    double output = controller.calculate(getAngle(), setpoint);
    output = Math.min(0.2, Math.max(-0.2, output));
    setOutput(output);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }

  private double getAngle() {
    return encoder.get() * 2 * Math.PI - zeroOffsetRads;
  }
}
