package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public abstract class ElevatorConfig {
  public String name = "Elevator";
  public double minAngleRads = Units.degreesToRadians(-90);
  public double maxAngleRads = Units.degreesToRadians(90);
  public double allowedErrorRads = Units.degreesToRadians(2);
  public ElevatorIO io;
}
