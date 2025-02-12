package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final SparkBase armMotor;
  private final AbsoluteEncoder armEncoder;
  private final PIDController pidController;

  // Set the PID constants (you’ll tune these values)
  private static final double kP = 0.1;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  public ArmSubsystem(int module) {
    armMotor =
        new SparkFlex(
            switch (module) {
              case 0 -> 9;
              default -> 0;
            },
            MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder();
    pidController = new PIDController(kP, kI, kD);
  }

  public void setArmPosition(double targetPosition) {
    // The PID controller computes the motor output to reach the target position
    double pidOutput = pidController.calculate(armEncoder.getPosition(), targetPosition);
    armMotor.set(pidOutput); // Set the motor to the calculated output
  }

  public void stop() {
    armMotor.set(0);
  }
}
