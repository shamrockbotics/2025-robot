package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean connected = false;
    public double targetAngleRads = 0.0;
    public double currentAngleRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the arm to the specified rotation. */
  public default void setPosition(double angleRads) {}

  /** Run open loop at the specified output in the range [-1, 1]. */
  public default void setOutput(double output) {}
}
