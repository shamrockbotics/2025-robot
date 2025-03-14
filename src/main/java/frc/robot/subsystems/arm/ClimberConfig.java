package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ClimberConfig extends ArmConfig {
  public ClimberConfig() {
    this(true);
  }

  public ClimberConfig(boolean real) {
    name = "Climber";
    minAngleRads = -0.62; // may need to double check
    maxAngleRads = 1.978;
    allowedErrorRads = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkMax(
              11, 12, -0.63, true, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 0.5, 0.0);
    } else {
      io = new ArmIOSim(minAngleRads, maxAngleRads, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
