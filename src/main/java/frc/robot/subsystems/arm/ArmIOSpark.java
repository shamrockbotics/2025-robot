package frc.robot.subsystems.arm;

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
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class ArmIOSpark implements ArmIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkBase spark;
  private final AbsoluteEncoder encoder;

  // Closed loop controllers
  private final SparkClosedLoopController controller;

  // Connection debouncers
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public ArmIOSpark(
      int id,
      Rotation2d zeroRotation,
      boolean motorInverted,
      boolean encoderInverted,
      double encoderPositionFactor,
      double encoderVelocityFactor,
      int currentLimit,
      double turnKp,
      double turnKd) {
    this.zeroRotation = zeroRotation;
    spark = new SparkMax(id, MotorType.kBrushless);
    encoder = spark.getAbsoluteEncoder();
    controller = spark.getClosedLoopController();

    SparkMaxConfig sparkConfig = new SparkMaxConfig();
    sparkConfig
        .inverted(motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
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
  public void updateInputs(ArmIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(spark, encoder::getPosition, (value) -> inputs.position = Rotation2d.fromRadians(value));
    ifOk(spark, encoder::getVelocity, (value) -> inputs.velocityRadPerSec = value);
    ifOk(
        spark,
        new DoubleSupplier[] {spark::getAppliedOutput, spark::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.connected = connectedDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(rotation.plus(zeroRotation).getRadians(), 0, 2 * Math.PI);
    controller.setReference(setpoint, ControlType.kPosition);
  }
}
