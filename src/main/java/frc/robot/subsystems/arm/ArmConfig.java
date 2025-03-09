package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public abstract class ArmConfig {
  public String name = "Arm";
  public double minAngleRads = Units.degreesToRadians(-90);
  public double maxAngleRads = Units.degreesToRadians(90);
  public double allowedErrorRads = Units.degreesToRadians(2);
  public ArmIO io;
}
