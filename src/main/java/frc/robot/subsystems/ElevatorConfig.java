package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.subsystems.mechanism.*;

public class ElevatorConfig extends MechanismConfig {
  public ElevatorConfig() {
    this(true);
  }

  public ElevatorConfig(boolean real) {
    name = "Elevator";
    motionType = Mechanism.MotionType.LINEAR;
    minPosition = 0;
    maxPosition = 1.3;
    allowedError = 0.01;
    if (real) {
      io =
          new MechanismIOSparkMax(
                  10,
                  false,
                  false,
                  2 * Math.PI * 0.0228 * 2,
                  2 * Math.PI * 0.0228 * 2 / 60,
                  40,
                  12.0,
                  3.0,
                  0.0)
              .addFollower(9, true)
              .addFeedforward(new ElevatorFeedforward(0, 2.0, 0));
    } else {
      io = new MechanismIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
