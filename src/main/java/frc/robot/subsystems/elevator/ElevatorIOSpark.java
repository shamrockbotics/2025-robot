package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final double zeroOffsetMeters;

  // Hardware objects
  private final SparkBase spark;
  private final AbsoluteEncoder encoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public ElevatorIOSpark(
      int id1,
      int id2,
      double zeroOffsetMeters,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double turnKp,
      double turnKd) {
    this(
        id1,
        zeroOffsetMeters,
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
        .inverted(!motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage)
        .follow(spark);
    tryUntilOk(
        followerSpark,
        5,
        () ->
            followerSpark.configure(
                followerSparkConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public ElevatorIOSpark(
      int id,
      double zeroOffsetMeters,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double turnKp,
      double turnKd) {
    this.zeroOffsetMeters = zeroOffsetMeters;
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = spark.getAbsoluteEncoder();
    controller = spark.getClosedLoopController();

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
    sparkConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, 2 * Math.PI)
        .pidf(turnKp, 0.0, turnKd, 0.0);
    sparkConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
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
    ifOk(spark, encoder::getPosition, (value) -> inputs.currentHeightMeters = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(double height) {
    double HeightMeters = height / .049;
    double setpoint = MathUtil.inputModulus(HeightMeters + zeroOffsetMeters, 0, 2 * Math.PI);
    controller.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }
}
