package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.MechanismIO;

public class ArmIOSim implements MechanismIO {
  private final SingleJointedArmSim armSim;
  private final DCMotorSim motorSim;
  private static int encoderPort = 0;
  private final Encoder encoder = new Encoder(encoderPort++, encoderPort++);
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  private DCMotor gearbox = DCMotor.getNEO(1);
  private PIDController controller = new PIDController(50, 0, 0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  double maxVoltage = 12.0;

  public ArmIOSim(
      double minPosition,
      double maxPosition,
      double gearReduction,
      double radsPerPulse,
      double lengthMeters,
      double massKg) {
    double moi = SingleJointedArmSim.estimateMOI(lengthMeters, massKg);
    armSim =
        new SingleJointedArmSim(
            gearbox,
            gearReduction,
            moi,
            lengthMeters,
            minPosition,
            maxPosition,
            false,
            0,
            radsPerPulse,
            0.0);
    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, gearReduction), gearbox);
    encoder.setDistancePerPulse(radsPerPulse);
  }

  @Override
  public void updateInputs(MechanismIOInputs inputs) {

    if (closedLoop) {
      appliedVolts = controller.calculate(encoder.getDistance());
    } else {
      controller.reset();
      controller.setSetpoint(encoder.getDistance());
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(0.02);

    armSim.setInputVoltage(appliedVolts);
    armSim.update(0.02);

    encoderSim.setDistance(armSim.getAngleRads());

    inputs.connected = true;
    inputs.targetPosition = controller.getSetpoint();
    inputs.currentPosition = encoder.getDistance();
    inputs.velocity = armSim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = armSim.getCurrentDrawAmps();
  }

  @Override
  public void setPosition(double angleRads) {
    closedLoop = true;
    controller.setSetpoint(angleRads);
  }

  @Override
  public void setOutput(double output) {
    closedLoop = false;
    appliedVolts = output * maxVoltage;
  }
}
