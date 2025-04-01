package frc.robot.subsystems.winch;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WinchIOSim implements WinchIO {
  private final DCMotorSim motorSim;

  private DCMotor gearbox = DCMotor.getNEO(1);
  private PIDController controller = new PIDController(50, 0, 0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  double maxVoltage = 12.0;

  public WinchIOSim(double gearReduction, double radsPerPulse, double lengthMeters, double massKg) {
    double moi = SingleJointedArmSim.estimateMOI(lengthMeters, massKg);

    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, gearReduction), gearbox);
  }

  @Override
  public void updateInputs(WinchIOInputs inputs) {

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    inputs.connected = true;
    inputs.velocity = motorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.appliedOutput = appliedVolts / maxVoltage;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
  }

  @Override
  public void setOutput(double output) {
    closedLoop = false;
    appliedVolts = output * maxVoltage;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    closedLoop = false;
    appliedVolts = voltage.in(Volts);
  }
}
