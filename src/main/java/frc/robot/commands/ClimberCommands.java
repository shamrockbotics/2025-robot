package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mechanism.Mechanism;
import frc.robot.subsystems.winch.Winch;

public class ClimberCommands {
  private Mechanism climberArm;
  private Winch climberWinch;

  private final double armStowPosition = 0.3;
  private final double armGrabPosition = 0.0;
  private final double armClimbPosition = 1.5;

  public ClimberCommands(Mechanism climberArm, Winch climberWinch) {
    this.climberArm = climberArm;
    this.climberWinch = climberWinch;
  }

  public Command stow() {
    return Commands.parallel(
            climberArm.runToPositionCommand(armStowPosition), climberWinch.stopCommand())
        .withName("Stow");
  }

  public Command grab() {
    return Commands.parallel(
            climberArm.runToPositionCommand(armGrabPosition), climberWinch.stopCommand())
        .withName("Grab");
  }

  public Command climb() {
    return Commands.parallel(
            climberArm.runToPositionCommand(armClimbPosition), climberWinch.winchCommand())
        .withName("Climb");
  }

  public Command release() {
    return Commands.parallel(
            climberArm.runToPositionCommand(armStowPosition), climberWinch.releaseCommand())
        .withName("Release");
  }
  
}
