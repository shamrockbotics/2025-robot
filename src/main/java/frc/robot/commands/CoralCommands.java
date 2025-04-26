package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.roller.Roller;

public class CoralCommands {
  private Mechanism elevator;
  private Mechanism coralElbow;
  private Mechanism coralWrist;
  private Roller coralIntake;

  public CoralCommands(
      Mechanism elevator, Mechanism coralElbow, Mechanism coralWrist, Roller coralIntake) {
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
            coralElbow.runToPositionCommand(Math.PI / 2),
            coralWrist.runToPositionCommand(0))
        .until(() -> onTarget())
        .withName("Stow");
  }

  public Command coralStation() {
    return Commands.parallel(
            elevator.runToPositionCommand(0.01),
            coralElbow.runToPositionCommand(1.72),
            coralWrist.runToPositionCommand(-0.7))
        .until(() -> onTarget())
        .withName("Coral Station");
  }

  public Command floor() {
    return Commands.parallel(
            elevator.runToPositionCommand(0),
            coralElbow.runToPositionCommand(0 + Math.PI / 2),
            coralWrist.runToPositionCommand(0))
        .until(() -> onTarget())
        .withName("Floor");
  }

  public Command l1() {
    return Commands.parallel(
            elevator.runToPositionCommand(0),
            coralElbow.runToPositionCommand(1.75),
            coralWrist.runToPositionCommand(-1.75))
        .until(() -> onTarget())
        .withName("L1");
  }

  public Command l2() {
    return Commands.parallel(
            elevator.runToPositionCommand(0.18),
            coralElbow.runToPositionCommand(1.71),
            coralWrist.runToPositionCommand(-1.49))
        .until(() -> onTarget())
        .withName("L2");
  }

  public Command l3() {
    return Commands.parallel(
            elevator.runToPositionCommand(0.6),
            coralElbow.runToPositionCommand(1.71),
            coralWrist.runToPositionCommand(-1.53))
        .until(() -> onTarget())
        .withName("L3");
  }

  public Command l4() {
    return Commands.parallel(
            elevator.runToPositionCommand(1.25),
            coralElbow.runToPositionCommand(1.87),
            coralWrist.runToPositionCommand(-1.57))
        .until(() -> onTarget())
        .withName("L4");
  }

  public Command l4Auto() {
    return Commands.parallel(
            elevator.runToPositionCommand(1.25),
            coralElbow.runToPositionCommand(1.67),
            coralWrist.runToPositionCommand(-1.54))
        .until(() -> onTarget())
        .withName("L4");
  }

  public Command climb() {
    return Commands.parallel(
            elevator.runToPositionCommand(0.18),
            coralElbow.runToPositionCommand(-0.07),
            coralWrist.runToPositionCommand(-1.53))
        .until(() -> onTarget())
        .withName("Climb");
  }

  public Command intake() {
    return coralIntake.intakeCommand();
  }

  public Command release() {
    return coralIntake.releaseCommand();
  }
}
