package frc.robot.subsystems;

import frc.robot.subsystems.winch.*;

public class ClimberWinchConfig extends WinchConfig {
  public ClimberWinchConfig() {
    this(true);
  }

  public ClimberWinchConfig(boolean real) {
    name = "Climber Winch";
    if (real) {
      io =
          new WinchIOSparkMax(11, false, true, 100 * 2 * Math.PI, 100 * 2 * Math.PI / 60, 40, 12.0)
              .addFollower(12, true)
              .addServo(9, 0.5, 0.0);
    } else {
      io = new WinchIOSim(200, (2.0 * Math.PI / 4096), 1.0, 1.0);
    }
  }
}
