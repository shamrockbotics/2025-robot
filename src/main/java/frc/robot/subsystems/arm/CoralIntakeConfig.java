package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class CoralIntakeConfig extends ArmConfig {
  public CoralIntakeConfig() {
    this(true);
  }

  public CoralIntakeConfig(boolean real) {
    name = "Algae Intake";
    minAngleRads = 0;
    maxAngleRads = 2;
    allowedErrorRads = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkRIO(
              15, 13, 0, 1.2, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io = new ArmIOSim(minAngleRads, maxAngleRads, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
