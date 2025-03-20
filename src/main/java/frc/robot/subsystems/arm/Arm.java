package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final double minAngleRads;
  private final double maxAngleRads;
  private final double allowedErrorRads;
  private final Alert disconnectedAlert;

  private boolean holding = false;
  private double setpoint = 0.0;
  private double lastRunAngle = 0.0;

  public MechanismLigament2d visualization;
  public DoubleSupplier visualizationAngleOffset = () -> 0.0;
  public boolean visualizationReversed = false;

  public Arm(ArmConfig config) {
    setName(config.name);
    io = config.io;

    minAngleRads = config.minAngleRads;
    maxAngleRads = config.maxAngleRads;
    allowedErrorRads = config.allowedErrorRads;

    disconnectedAlert = new Alert(getName() + " disconnected.", AlertType.kError);

    visualization = new MechanismLigament2d(getName(), 1, lastRunAngle);

    setDefaultCommand(holdCommand());

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);
    inputs.onTarget = (Math.abs(getAngle() - setpoint) <= allowedErrorRads);

    if (!holding || DriverStation.isDisabled()) {
      if (onTarget()) {
        lastRunAngle = setpoint;
      } else {
        lastRunAngle = getAngle();
      }
    }

    double visualizationAngle =
        Units.radiansToDegrees(getAngle()) + visualizationAngleOffset.getAsDouble();
    if (visualizationReversed) visualizationAngle = 180 - visualizationAngle;
    visualization.setAngle(visualizationAngle);

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the arm to the desired angle.
   *
   * @param angleRads Angle in radians
   */
  public void runToAngle(double angleRads) {
    if (holding) inputs.onTarget = false;
    setpoint = angleRads;
    holding = false;
    io.setPosition(MathUtil.clamp(angleRads, minAngleRads, maxAngleRads));
  }

  /**
   * Runs the arm motor at the desired percent.
   *
   * @param value Output value, -1 to +1, + output moves in direction of + angle
   */
  public void run(double value) {
    holding = false;
    if (value < 0.0 && getAngle() <= minAngleRads) {
      io.setOutput(0.0);
    } else if (value > 0.0 && getAngle() >= maxAngleRads) {
      io.setOutput(0.0);
    } else {
      io.setOutput(value);
    }
  }

  /** Disables all outputs to motors. */
  public void stop() {
    holding = false;
    io.setOutput(0.0);
  }

  /** Holds the arm at the last run angle. */
  public void hold() {
    holding = true;
    io.setPosition(lastRunAngle);
  }

  /** Returns the current angle of the arm in radians. */
  public double getAngle() {
    return inputs.currentAngleRads;
  }

  /** Returns true if the arm angle is within the allowed error of the target angle. */
  public boolean onTarget() {
    return inputs.onTarget;
  }

  public Command runToPositionCommand(double position) {
    return run(() -> runToAngle(position)).withName("Run To Position " + position);
  }

  public Command holdCommand() {
    return run(() -> hold()).withName("Hold");
  }

  public Command runPercentCommand(DoubleSupplier valueSupplier) {
    double value = valueSupplier.getAsDouble();
    return run(() -> run(value)).withName("Run Percent " + value);
  }
}
