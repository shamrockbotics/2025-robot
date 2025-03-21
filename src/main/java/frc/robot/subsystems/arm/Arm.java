package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Mechanism;
import frc.robot.subsystems.MechanismConfig;
import java.util.function.DoubleSupplier;

public class Arm extends Mechanism {
  public DoubleSupplier visualizationAngleOffset = () -> 0.0;
  public boolean visualizationReversed = false;

  public Arm(MechanismConfig config) {
    super(config);
  }

  protected void updateVisualization() {
    double visualizationAngle =
        Units.radiansToDegrees(getPosition()) + visualizationAngleOffset.getAsDouble();
    if (visualizationReversed) visualizationAngle = 180 - visualizationAngle;
    visualization.setAngle(visualizationAngle);
  }
}
