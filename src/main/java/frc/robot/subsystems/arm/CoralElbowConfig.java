package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.MechanismConfig;
import frc.robot.subsystems.MechanismIOSim;

public class CoralElbowConfig extends MechanismConfig {
  public CoralElbowConfig() {
    this(true);
  }

  public CoralElbowConfig(boolean real) {
    name = "Coral Elbow";
    minPosition = -2.4;
    maxPosition = 0.05;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkMax(
              13, 3.05, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 6.0, 2.0, 0.0);
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
