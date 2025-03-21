package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;

public abstract class MechanismConfig {
  public String name = "Arm";
  public double minPosition = Units.degreesToRadians(-90);
  public double maxPosition = Units.degreesToRadians(90);
  public double allowedError = Units.degreesToRadians(2);
  public MechanismIO io;
}
