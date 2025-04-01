package frc.robot.subsystems.winch;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Winch extends SubsystemBase {
  private final WinchIO io;
  private final WinchIOInputsAutoLogged inputs = new WinchIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private final Alert disconnectedAlert;

  private boolean holding = false;
  private double setpoint = 0.0;
  private double lastRunPosition = 0.0;

  public MechanismLigament2d visualization;
  public DoubleSupplier visualizationAngleOffset = () -> 0.0;
  public DoubleSupplier visualizationLengthOffset = () -> 0.0;
  public boolean visualizationReversed = false;

  public static enum MotionType {
    LINEAR,
    ANGULAR
  }

  public Winch(WinchConfig config) {
    setName(config.name);
    io = config.io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(getName() + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltage, null, this));

    disconnectedAlert = new Alert(getName() + " disconnected.", AlertType.kError);

    visualization = new MechanismLigament2d(getName(), 1, lastRunPosition);

    setDefaultCommand(stopCommand());

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(this.getName(), inputs);

    disconnectedAlert.set(!inputs.connected);
  }

  /**
   * Runs the motor at the desired percent.
   *
   * @param value Output value, -1 to +1
   */
  public void run(double value) {
    io.setOutput(value);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setOutput(0.0);
  }

  public Command stopCommand() {
    return run(() -> stop()).withName("Stop");
  }

  public Command runPercentCommand(DoubleSupplier valueSupplier) {
    return run(() -> run(valueSupplier.getAsDouble())).withName("Run Percent");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public void addSysIdCommands(LoggedDashboardChooser<Command> chooser) {
    chooser.addOption(
        getName() + " SysId (Quasistatic Forward)",
        sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        getName() + " SysId (Quasistatic Reverse)",
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    chooser.addOption(
        getName() + " SysId (Dynamic Forward)", sysIdDynamic(SysIdRoutine.Direction.kForward));
    chooser.addOption(
        getName() + " SysId (Dynamic Reverse)", sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
