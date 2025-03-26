package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class AlgaeArmConfig extends MechanismConfig {
  public AlgaeArmConfig() {
    this(true);
  }

  public AlgaeArmConfig(boolean real) {
    name = "Algae Arm";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -2.0;
    maxPosition = 0.5;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
              15, 3.08, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 0.5, 0.0);
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
