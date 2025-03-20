package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;

public class CoralCommands {
  private Elevator elevator;
  private Arm coralElbow;
  private Arm coralWrist;
  private Roller coralIntake;

  public CoralCommands(Elevator elevator, Arm coralElbow, Arm coralWrist, Roller coralIntake) {
    this.elevator = elevator;
    this.coralElbow = coralElbow;
    this.coralWrist = coralWrist;
    this.coralIntake = coralIntake;
  }

  private boolean onTarget() {
    return elevator.onTarget() && coralElbow.onTarget() && coralWrist.onTarget();
  }

  public Command stow() {
    return Commands.parallel(
            elevator.runToPositionCommand(0),
            coralElbow.runToPositionCommand(0),
            coralWrist.runToPositionCommand(-0.6))
        .until(() -> onTarget())
        .withName("Stow");
  }

  public Command coralStation() {
    return Commands.parallel(
            elevator.runToPositionCommand(0.13),
            coralElbow.runToPositionCommand(-0.7),
            coralWrist.runToPositionCommand(0.1))
        .until(() -> onTarget())
        .withName("Coral Station");
  }

  public Command floor() {
    return Commands.parallel(
            elevator.runToPositionCommand(0),
            coralElbow.runToPositionCommand(0),
            coralWrist.runToPositionCommand(0))
        .until(() -> onTarget())
        .withName("Floor");
  }

  public Command l1() {
    return Commands.parallel(
            elevator.runToPositionCommand(0),
            coralElbow.runToPositionCommand(-0.15),
            coralWrist.runToPositionCommand(-1.3))
        .until(() -> onTarget())
        .withName("L1");
  }

  public Command l2() {
    return Commands.parallel(
            elevator.runToPositionCommand(0.65),
            coralElbow.runToPositionCommand(-0.79),
            coralWrist.runToPositionCommand(-1.4))
        .until(() -> onTarget())
        .withName("L2");
  }

  public Command l3() {
    return Commands.parallel(
            elevator.runToPositionCommand(1.0),
            coralElbow.runToPositionCommand(-0.79),
            coralWrist.runToPositionCommand(-1.2))
        .until(() -> onTarget())
        .withName("L3");
  }

  public Command l4() {
    return Commands.parallel(
            elevator.runToPositionCommand(1.25),
            coralElbow.runToPositionCommand(-0.15),
            coralWrist.runToPositionCommand(-1.3))
        .until(() -> onTarget())
        .withName("L4");
  }

  public Command intake() {
    return coralIntake.intakeCommand();
  }

  public Command release() {
    return coralIntake.releaseCommand();
  }
}
