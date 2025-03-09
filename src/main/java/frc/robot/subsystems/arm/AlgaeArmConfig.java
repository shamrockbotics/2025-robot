package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class AlgaeArmConfig extends ArmConfig {
  public AlgaeArmConfig() {
    this(true);
  }

  public AlgaeArmConfig(boolean real) {
    name = "Algae Arm";
    minAngleRads = 0;
    maxAngleRads = 2;
    allowedErrorRads = Units.degreesToRadians(2);
    if (real) {
      io =
          new ArmIOSpark(
              15, 13, 0.0, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io = new ArmIOSim(minAngleRads, maxAngleRads, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
