package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean connected = false;
    public Rotation2d target = new Rotation2d();
    public Rotation2d position = new Rotation2d();
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the arm to the specified rotation. */
  public default void setPosition(Rotation2d rotation) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
