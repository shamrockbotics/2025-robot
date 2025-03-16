package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.*;

public class CoralElbowCommands {
  private CoralElbowCommands() {}
  ;

  public static Command setL4(Arm arm) {
    return Commands.run(
        () -> {
          arm.runToAngle(0);
        },
        arm);
  }
  ;

  public static Command holdPosition(Arm arm) {
    return Commands.run(
        () -> {
          arm.hold();
        },
        arm);
  }
}
;
