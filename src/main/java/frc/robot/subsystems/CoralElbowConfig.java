package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.mechanism.*;

public class CoralElbowConfig extends MechanismConfig {
  public CoralElbowConfig() {
    this(true);
  }

  public CoralElbowConfig(boolean real) {
    name = "Coral Elbow";
    motionType = Mechanism.MotionType.ANGULAR;
    minPosition = -2.4 + Math.PI / 2;
    maxPosition = 2.1;
    allowedError = Units.degreesToRadians(2);
    if (real) {
      io =
          new MechanismIOSparkMax(
                  13, 1.63, false, true, 2 * Math.PI, 2 * Math.PI / 60, 40, 3.0, 1.7, 0.0)
              .addFeedforward(new ArmFeedforward(0, 1.3, 0));
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
