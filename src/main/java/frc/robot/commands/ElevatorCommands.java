package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.*;

public class ElevatorCommands {
  private ElevatorCommands() {}

  public static Command runToL4(Elevator elevator) {
    return Commands.run(
            () -> {
              elevator.runToHeight(1.25);
            },
            elevator)
        .until(() -> elevator.onTarget());
  }
  public static Command runToL3(Elevator elevator) {
    return Commands.run(
            () -> {
              elevator.runToHeight(1.05);
            },
            elevator)
        .until(() -> elevator.onTarget());
  }
  public static Command runToL2(Elevator elevator) {
    return Commands.run(
            () -> {
              elevator.runToHeight(.65);
            },
            elevator)
        .until(() -> elevator.onTarget());
  }
  public static Command runToL1(Elevator elevator) {
    return Commands.run(
            () -> {
              elevator.runToHeight(0);
            },
            elevator)
        .until(() -> elevator.onTarget());
  }


  public static Command holdPosition(Elevator elevator) {
    return Commands.run(
        () -> {
          elevator.hold();
        },
        elevator);
  }
}
