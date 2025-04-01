package frc.robot.subsystems.winch;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WinchIO {
  @AutoLog
  public static class WinchIOInputs {
    public boolean connected = false;
    public double velocity = 0.0;
    public double appliedVolts = 0.0;
    public double appliedOutput = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(WinchIOInputs inputs) {}

  /** Run open loop at the specified output in the range [-1, 1]. */
  public default void setOutput(double output) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(Voltage voltage) {}
}
