package frc.robot.subsystems.elevator;

public class ElevatorSpecificConfig extends ElevatorConfig {
  public ElevatorSpecificConfig() {
    this(true);
  }

  public ElevatorSpecificConfig(boolean real) {
    name = "Elevator";
    minHeightMeters = 0;
    maxHeightMeters = 2;
    allowedErrorMeters = 0.02;
    if (real) {
      io =
          new ElevatorIOSparkMax(
              10,
              9,
              false,
              false,
              2 * Math.PI * 0.0222 * 2,
              2 * Math.PI * 0.0222 * 2 / 60,
              40,
              2.0,
              0.0);
    } else {
      io =
          new ElevatorIOSim(
              minHeightMeters, maxHeightMeters, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
