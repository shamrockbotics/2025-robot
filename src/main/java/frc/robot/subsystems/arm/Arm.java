package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert disconnectedAlert;

  public Arm(ArmIO io) {
    this("Arm", io);
  }

  public Arm(String name, ArmIO io) {
    this.setName(name);
    this.io = io;

    disconnectedAlert = new Alert(name + " disconnected.", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the arm to the desired angle.
   *
   * @param angle Angle in Rotation2d
   */
  public void runToAngle(Rotation2d angle) {
    io.setPosition(angle);
  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setVoltage(0.0);
  }

  /** Holds the arm at the last run angle. */
  public void hold() {
    io.setPosition(inputs.target);
  }

  /** Returns the current turn angle of the arm. */
  public Rotation2d getAngle() {
    return inputs.position;
  }

  public boolean onTarget(Rotation2d allowedError) {
    return onTarget(allowedError.getRadians());
  }

  public boolean onTarget() {
    return onTarget(Rotation2d.fromDegrees(5));
  }

  public boolean onTarget(double allowedErrorRads) {
    return (Math.abs(inputs.position.minus(inputs.target).getRadians()) <= allowedErrorRads);
  }
}
