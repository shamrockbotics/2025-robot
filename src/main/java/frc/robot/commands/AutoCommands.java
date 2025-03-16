package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.roller.*;

// Configure MyPath
public class AutoCommands {
  private AutoCommands() {}
  ;

  public static Command L4(
      Elevator elevator, Roller roller, Arm coralElbow, Arm coralWrist, Drive drive) {
    return Commands.sequence(
        DriveCommands.joystickDrive(drive, () -> 0.4, () -> 0, () -> 0, false).withTimeout(4.0),
        DriveCommands.joystickDrive(drive, () -> -0.4, () -> 0, () -> 0, false).withTimeout(0.5),
        DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, false).withTimeout(0.1),
        ElevatorCommands.runToL4(elevator),
        CoralElbowCommands.setL4(coralElbow),
        CoralWristCommands.setL4(coralWrist),
        Commands.waitSeconds(0.5),
        CoralIntakeCommands.extake(roller));
  }

  public static Command L3(
      Elevator elevator, Roller roller, Arm coralElbow, Arm coralWrist, Drive drive) {
    return Commands.sequence(
        DriveCommands.joystickDrive(drive, () -> 0.4, () -> 0, () -> 0, false).withTimeout(4.0),
        DriveCommands.joystickDrive(drive, () -> -0.4, () -> 0, () -> 0, false).withTimeout(0.5),
        DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, false).withTimeout(0.1),
        ElevatorCommands.runToL3(elevator),
        CoralElbowCommands.setL3(coralElbow),
        CoralWristCommands.setL3(coralWrist),
        Commands.waitSeconds(0.5),
        CoralIntakeCommands.extake(roller));
  }
  ;

  public static Command L2(
      Elevator elevator, Roller roller, Arm coralElbow, Arm coralWrist, Drive drive) {
    return Commands.sequence(
        DriveCommands.joystickDrive(drive, () -> 0.4, () -> 0, () -> 0, false).withTimeout(4.0),
        DriveCommands.joystickDrive(drive, () -> -0.4, () -> 0, () -> 0, false).withTimeout(0.5),
        DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, false).withTimeout(0.1),
        ElevatorCommands.runToL2(elevator),
        CoralElbowCommands.setL2(coralElbow),
        CoralWristCommands.setL2(coralWrist),
        Commands.waitSeconds(0.5),
        CoralIntakeCommands.extake(roller));
  }
  ;

  public static Command L1(
      Elevator elevator, Roller roller, Arm coralElbow, Arm coralWrist, Drive drive) {
    return Commands.sequence(
        DriveCommands.joystickDrive(drive, () -> 0.4, () -> 0, () -> 0, false).withTimeout(4.0),
        DriveCommands.joystickDrive(drive, () -> -0.4, () -> 0, () -> 0, false).withTimeout(0.5),
        DriveCommands.joystickDrive(drive, () -> 0, () -> 0, () -> 0, false).withTimeout(0.1),
        ElevatorCommands.runToL1(elevator),
        CoralElbowCommands.setL1(coralElbow),
        CoralWristCommands.setL1(coralWrist),
        Commands.waitSeconds(0.5),
        CoralIntakeCommands.extake(roller));
  }
  ;
}
