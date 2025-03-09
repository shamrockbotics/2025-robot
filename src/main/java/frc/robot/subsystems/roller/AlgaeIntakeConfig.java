package frc.robot.subsystems.roller;

public class AlgaeIntakeConfig extends RollerConfig {
  public AlgaeIntakeConfig() {
    this(true);
  }

  public AlgaeIntakeConfig(boolean real) {
    name = "Algae Intake";
    if (real) {
      io = new RollerIOSparkMax(16, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
    }
  }
}
