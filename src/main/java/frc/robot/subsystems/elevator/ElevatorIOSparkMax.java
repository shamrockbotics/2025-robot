package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.MechanismIO;
import java.util.function.DoubleSupplier;

public class ElevatorIOSparkMax implements MechanismIO {
  // Hardware objects
  private final SparkMax spark;
  private final RelativeEncoder encoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;
  private double setpoint = 0.0;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  double maxVoltage = 12.0;

  public ElevatorIOSparkMax(
      int id1,
      int id2,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double positionKp,
      double positionKd) {
    this(
        id1,
        motorInverted,
        encoderInverted,
        encoderPositionFactor,
        encoderVelocityFactor,
        currentLimit,
        positionKp,
        positionKd);
    SparkMax followerSpark = new SparkMax(id2, MotorType.kBrushless);
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
  }

  public ElevatorIOSparkMax(
      int id,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double positionKp,
      double positionKd) {
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = spark.getAlternateEncoder();
    controller = spark.getClosedLoopController();

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(maxVoltage);
    sparkConfig
        .alternateEncoder
        .inverted(encoderInverted)
        .positionConversionFactor(encoderPositionFactor)
        .velocityConversionFactor(encoderVelocityFactor)
        .averageDepth(2)
        .countsPerRevolution(8192)
        .setSparkMaxDataPortConfig();
    sparkConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pidf(positionKp, 0.0, positionKd, 0.0);
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
  public void updateInputs(MechanismIOInputs inputs) {
    if (encoder.getPosition() < 0.0) encoder.setPosition(0.0);

    // Update inputs
    sparkStickyFault = false;
    ifOk(spark, encoder::getPosition, (value) -> inputs.currentPosition = value);
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.targetPosition = setpoint;
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(double height) {
    setpoint = height;
    controller.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }
}
