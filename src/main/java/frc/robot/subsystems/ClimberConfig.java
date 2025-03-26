package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class ClimberConfig extends MechanismConfig {
  public ClimberConfig() {
    this(true);
  }

  public ClimberConfig(boolean real) {
    name = "Climber";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -0.62; // may need to double check
    maxPosition = 1.978;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
                  11, -0.63, true, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 12.0, 0.5, 0.0)
              .addFollower(12, true);
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
