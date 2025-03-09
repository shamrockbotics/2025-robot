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
          new ElevatorIOSpark(
              9, 10, 0.0, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5, 0.0);
    } else {
      io =
          new ElevatorIOSim(
              minHeightMeters, maxHeightMeters, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
