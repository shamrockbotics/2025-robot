package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class ClimberArmConfig extends MechanismConfig {
  public ClimberArmConfig() {
    this(true);
  }

  public ClimberArmConfig(boolean real) {
    name = "Climber Arm";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -Math.PI;
    maxPosition = Math.PI;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
              15, 0.0, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 0.05, 0.0);
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
