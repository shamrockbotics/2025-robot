package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.roller.*;
import com.pathplanner.lib.commands.PathPlannerAuto;
//Configure MyPath
public class AutoCommands {
    private AutoCommands(){};
    public static Command L4(Elevator elevator, Roller roller, Arm coralElbow, Arm coralWrist){
        return Commands.sequence(
            new PathPlannerAuto("vlad"),
            ElevatorCommands.runToL4(elevator),
            ElevatorCommands.holdPosition(elevator),
            CoralElbowCommands.setL4(coralElbow),
            CoralElbowCommands.holdPosition(coralWrist),
            CoralWristCommands.setL4(coralWrist),
            CoralWristCommands.holdPosition(coralWrist),
            CoralIntakeCommands.extake(roller)
        );
    }
}
