package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.MechanismConfig;
import frc.robot.subsystems.MechanismIOSim;

public class ClimberConfig extends MechanismConfig {
  public ClimberConfig() {
    this(true);
  }

  public ClimberConfig(boolean real) {
    name = "Climber";
    minPosition = -0.62; // may need to double check
    maxPosition = 1.978;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkMax(
              11, 12, -0.63, true, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 0.5, 0.0);
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
