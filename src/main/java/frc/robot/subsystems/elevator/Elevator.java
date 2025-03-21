package frc.robot.subsystems.elevator;

import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.MechanismConfig;
import java.util.function.DoubleSupplier;

public class Elevator extends Mechanism {
  public DoubleSupplier visualizationHeightOffset = () -> 0.0;
  public boolean visualizationReversed = false;

  public Elevator(MechanismConfig config) {
    super(config);
  }

  @Override
  protected void updateVisualization() {
    visualization.setLength(getPosition() + visualizationHeightOffset.getAsDouble());
  }
}
