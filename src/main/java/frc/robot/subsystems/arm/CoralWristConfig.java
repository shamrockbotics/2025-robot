package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.MechanismConfig;

public class CoralWristConfig extends MechanismConfig {
  public CoralWristConfig() {
    this(true);
  }

  public CoralWristConfig(boolean real) {
    name = "Coral Wrist";
    minPosition = -1.9;
    maxPosition = 1.85;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkMax(14, 3.5, true, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 4.0, 2.0, 0.0);
    } else {
      io = new ArmIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
