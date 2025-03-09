package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final double minHeightMeters;
  private final double maxHeightMeters;
  private final double allowedErrorMeters;
  private final Alert disconnectedAlert;

  private double lastRunAngle = 0.0;

  public MechanismLigament2d visualization;
  public DoubleSupplier visualizationAngleOffset = () -> 0.0;

  public Elevator(ElevatorConfig config) {
    setName(config.name);
    io = config.io;

    minHeightMeters = config.minHeightMeters;
    maxHeightMeters = config.maxHeightMeters;
    allowedErrorMeters = config.allowedErrorMeters;

    disconnectedAlert = new Alert(getName() + " disconnected.", AlertType.kError);

    visualization = new MechanismLigament2d(getName(), 1, lastRunAngle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);

    visualization.setLength(getHeight());

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the arm to the desired angle.
   *
   * @param HeightMeters Angle in radians
   */
  public void runToHeight(double height) {
    io.setPosition(height);
    lastRunAngle = getHeight();
  }

  /**
   * Runs the arm to the desired angle.
   *
   * @param degrees Angle in degrees
   */
  public void runToDegrees(double degrees) {
    runToHeight(Units.degreesToRadians(degrees));
  }

  /**
   * Runs the arm motor at the desired percent.
   *
   * @param value Output value, -1 to +1, + output moves in direction of + angle
   */
  public void run(double value) {
    if (value < 0.0 && getHeight() <= minHeightMeters) {
      io.setOutput(0.0);
    } else if (value > 0.0 && getHeight() >= maxHeightMeters) {
      io.setOutput(0.0);
    } else {
      io.setOutput(value);
    }
    lastRunAngle = getHeight();
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setOutput(0.0);
    lastRunAngle = getHeight();
  }

  /** Holds the arm at the last run angle. */
  public void hold() {
    io.setPosition(lastRunAngle);
  }

  /** Returns the current angle of the arm in radians. */
  public double getHeight() {
    return inputs.currentHeightMeters;
  }

  /** Returns true if the arm angle is within the allowed error of the target angle. */
  public boolean onTarget() {
    return (Math.abs(inputs.currentHeightMeters - inputs.targetHeightMeters) <= allowedErrorMeters);
  }
}
