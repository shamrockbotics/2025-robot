package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final double minAngleRads;
  private final double maxAngleRads;
  private final double allowedErrorRads;
  private final Alert disconnectedAlert;

  private double lastRunAngle = 0.0;

  public Arm(ArmConfig config) {
    setName(config.name);
    io = config.io;

    minAngleRads = config.minAngleRads;
    maxAngleRads = config.maxAngleRads;
    allowedErrorRads = config.allowedErrorRads;

    disconnectedAlert = new Alert(config.name + " disconnected.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the arm to the desired angle.
   *
   * @param angleRads Angle in radians
   */
  public void runToAngle(double angleRads) {
    io.setPosition(MathUtil.clamp(angleRads, minAngleRads, maxAngleRads));
    lastRunAngle = getAngle();
  }

  /**
   * Runs the arm to the desired angle.
   *
   * @param degrees Angle in degrees
   */
  public void runToDegrees(double degrees) {
    runToAngle(Units.degreesToRadians(degrees));
  }

  /**
   * Runs the arm motor at the desired percent.
   *
   * @param value Output value, -1 to +1, + output moves in direction of + angle
   */
  public void run(double value) {
    if (value < 0.0 && getAngle() <= minAngleRads) {
      io.setOutput(0.0);
    } else if (value > 0.0 && getAngle() >= maxAngleRads) {
      io.setOutput(0.0);
    } else {
      io.setOutput(value);
    }
    lastRunAngle = getAngle();
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setOutput(0.0);
    lastRunAngle = getAngle();
  }

  /** Holds the arm at the last run angle. */
  public void hold() {
    io.setPosition(lastRunAngle);
  }

  /** Returns the current turn angle of the arm. */
  public double getAngle() {
    return inputs.currentAngleRads;
  }

  /** Returns true if the arm angle is within the allowed error of the target angle. */
  public boolean onTarget() {
    return (Math.abs(inputs.currentAngleRads - inputs.targetAngleRads) <= allowedErrorRads);
  }
}
