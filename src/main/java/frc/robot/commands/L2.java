// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L2 extends Command {
  private Elevator elevator;
  private Arm coralElbow;
  private Arm coralWrist;

  /** Creates a new L4. */
  public L2(Elevator elevator, Arm coralElbow, Arm coralWrist) {
    this.elevator = elevator;
    this.coralElbow = coralElbow;
    this.coralWrist = coralWrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coralElbow, coralWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runToHeight(.65);
    coralElbow.runToAngle(-.79);
    coralWrist.runToAngle(-1.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
