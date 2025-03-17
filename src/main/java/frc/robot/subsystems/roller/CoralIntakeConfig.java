package frc.robot.subsystems.roller;

public class CoralIntakeConfig extends RollerConfig {
  public CoralIntakeConfig() {
    this(true);
  }

  public CoralIntakeConfig(boolean real) {
    name = "Coral Intake";
    if (real) {
      io = new RollerIOSparkMax(17, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
