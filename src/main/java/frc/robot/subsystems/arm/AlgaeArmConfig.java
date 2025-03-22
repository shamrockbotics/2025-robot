package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class AlgaeArmConfig extends ArmConfig {
  public AlgaeArmConfig() {
    this(true);
  }

  public AlgaeArmConfig(boolean real) {
    name = "Algae Arm";
    minAngleRads = -2.0;
    maxAngleRads = 0.4;
    allowedErrorRads = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSparkMax(
              15, -2.016, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 10.0, 0.5, 0);
    } else {
      io = new ArmIOSim(minAngleRads, maxAngleRads, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
