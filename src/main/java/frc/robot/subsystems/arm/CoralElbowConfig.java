package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class CoralElbowConfig extends ArmConfig {
  public CoralElbowConfig() {
    this(true);
  }

  public CoralElbowConfig(boolean real) {
    name = "Coral Elbow";
    minAngleRads = -2.4;
    maxAngleRads = 0.05;
    allowedErrorRads = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkMax(
              13, 2.832, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 6.0, 2.0, 0.0);
    } else {
      io = new ArmIOSim(minAngleRads, maxAngleRads, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
