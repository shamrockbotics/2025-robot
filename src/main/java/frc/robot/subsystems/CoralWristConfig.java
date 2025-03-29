package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class CoralWristConfig extends MechanismConfig {
  public CoralWristConfig() {
    this(true);
  }

  public CoralWristConfig(boolean real) {
    name = "Coral Wrist";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -1.9;
    maxPosition = 1.85;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
                  14, -0.28, true, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 4.0, 2.0, 0.0)
              .addFeedforward(new ArmFeedforward(0, 0.5, 0));
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
