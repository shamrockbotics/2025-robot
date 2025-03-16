package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.*;

public class ElevatorCommands {
  private ElevatorCommands() {}
  ;

  public static Command runToL4(Elevator elevator) {
    return Commands.run(
        () -> {
          elevator.runToHeight(1.25);
        },
        elevator);
  }
  ;

  public static Command holdPosition(Elevator elevator) {
    return Commands.run(
        () -> {
          elevator.hold();
        },
        elevator);
  }
}
;
