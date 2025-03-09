package frc.robot.subsystems.elevator;

public abstract class ElevatorConfig {
  public String name = "Elevator";
  public double minHeightMeters = 0;
  public double maxHeightMeters = 2;
  public double allowedErrorMeters = 0.02;
  public ElevatorIO io;
}
