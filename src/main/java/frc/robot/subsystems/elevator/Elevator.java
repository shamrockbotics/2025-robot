package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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

  private boolean holding = false;
  private double lastRunHeight = 0.0;

  public MechanismLigament2d visualization;
  public DoubleSupplier visualizationHeightOffset = () -> 0.0;

  public Elevator(ElevatorConfig config) {
    setName(config.name);
    io = config.io;

    minHeightMeters = config.minHeightMeters;
    maxHeightMeters = config.maxHeightMeters;
    allowedErrorMeters = config.allowedErrorMeters;

    disconnectedAlert = new Alert(getName() + " disconnected.", AlertType.kError);

    visualization = new MechanismLigament2d(getName(), 1, lastRunHeight);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);

    if (!holding || DriverStation.isDisabled()) {
      lastRunHeight = getHeight();
    }

    visualization.setLength(getHeight() + visualizationHeightOffset.getAsDouble());
    visualization.setColor(new Color8Bit(Color.kGoldenrod));

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the arm to the desired height.
   *
   * @param height Height in meters
   */
  public void runToHeight(double height) {
    holding = false;
    io.setPosition(height);
  }

  /**
   * Runs the motor at the desired percent.
   *
   * @param value Output value, -1 to +1, + output extends elevator
   */
  public void run(double value) {
    holding = false;
    if (value < 0.0 && getHeight() <= minHeightMeters) {
      io.setOutput(0.0);
    } else if (value > 0.0 && getHeight() >= maxHeightMeters) {
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

  /** Holds the arm at the last run height. */
  public void hold() {
    holding = true;
    io.setPosition(lastRunHeight);
  }

  /** Returns the current height of the arm in meters. */
  public double getHeight() {
    return inputs.currentHeightMeters;
  }

  /** Returns true if the arm height is within the allowed error of the target height. */
  public boolean onTarget() {
    return (Math.abs(inputs.currentHeightMeters - inputs.targetHeightMeters) <= allowedErrorMeters);
  }
}
