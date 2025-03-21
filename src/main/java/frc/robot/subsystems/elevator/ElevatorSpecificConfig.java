package frc.robot.subsystems.elevator;

import frc.robot.subsystems.MechanismConfig;

public class ElevatorSpecificConfig extends MechanismConfig {
  public ElevatorSpecificConfig() {
    this(true);
  }

  public ElevatorSpecificConfig(boolean real) {
    name = "Elevator";
    minPosition = 0;
    maxPosition = 1.3;
    allowedError = 0.02;
    if (real) {
      io =
          new ElevatorIOSparkMax(
              10,
              9,
              false,
              false,
              2 * Math.PI * 0.0228 * 2,
              2 * Math.PI * 0.0228 * 2 / 60,
              40,
              10.0,
              0.0);
    } else {
      io = new ElevatorIOSim(minPosition, maxPosition, 200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
