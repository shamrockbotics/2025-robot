package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.roller.*;

public class CoralIntakeCommands {
  private CoralIntakeCommands() {}
  ;

  public static Command intake(Roller roller) {
    return Commands.run(
        () -> {
          roller.run(.2);
        },
        roller);
  }
  ;

  public static Command extake(Roller roller) {
    return Commands.run(
        () -> {
          roller.run(-.2);
        },
        roller);
  }
  ;
}
;
