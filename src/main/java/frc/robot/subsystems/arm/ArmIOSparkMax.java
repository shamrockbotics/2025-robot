package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.subsystems.MechanismIO;
import java.util.function.DoubleSupplier;

public class ArmIOSparkMax implements MechanismIO {
  private final double zeroOffset;

  // Hardware objects
  private final SparkMax spark;
  private final AbsoluteEncoder encoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;
  private double setpoint = 0.0;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  double maxVoltage = 12.0;

  public ArmIOSparkMax(
      int id1,
      int id2,
      double zeroOffset,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double voltageLimit,
      double positionKp,
      double positionKd) {
    this(
        id1,
        zeroOffset,
        motorInverted,
        encoderInverted,
        encoderPositionFactor,
        encoderVelocityFactor,
        currentLimit,
        voltageLimit,
        positionKp,
        positionKd);
    SparkMax followerSpark = new SparkMax(id2, MotorType.kBrushless);
    SparkMaxConfig followerSparkConfig = new SparkMaxConfig();
    followerSparkConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(voltageLimit)
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

  public ArmIOSparkMax(
      int id,
      double zeroOffset,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double voltageLimit,
      double positionKp,
      double positionKd) {
    this.zeroOffset = zeroOffset;
    maxVoltage = voltageLimit;
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
    // Update inputs
    sparkStickyFault = false;
    ifOk(
        spark,
        encoder::getPosition,
        (value) ->
            inputs.currentPosition = MathUtil.inputModulus(value - zeroOffset, -Math.PI, Math.PI));
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.targetPosition = MathUtil.inputModulus(setpoint - zeroOffset, -Math.PI, Math.PI);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(double angleRads) {
    setpoint = MathUtil.inputModulus(angleRads + zeroOffset, 0, 2 * Math.PI);
    controller.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setOutput(double value) {
    spark.setVoltage(value * maxVoltage);
  }
}
