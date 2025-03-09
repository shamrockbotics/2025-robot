package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public double targetAngleRads = 0.0;
    public double currentAngleRads = 0.0;
    public double currentHeight = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public double maxVoltage = 12.0;

  /** Update the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the arm to the specified rotation. */
  public default void setPosition(double height) {}

  /** Run open loop at the specified output in the range [-1, 1]. */
  public default void setOutput(double output) {}
}
