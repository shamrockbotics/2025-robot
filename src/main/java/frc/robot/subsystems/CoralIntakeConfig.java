package frc.robot.subsystems;

import frc.robot.subsystems.roller.RollerConfig;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.roller.RollerIOSparkMax;

public class CoralIntakeConfig extends RollerConfig {
  public CoralIntakeConfig() {
    this(true);
  }

  public CoralIntakeConfig(boolean real) {
    name = "Coral Intake";
    intakePercent = 0.3;
    releasePercent = 0.3;
    if (real) {
      io = new RollerIOSparkMax(17, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
