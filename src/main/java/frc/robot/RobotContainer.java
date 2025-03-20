// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.roller.*;
import frc.robot.subsystems.vision.*;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Arm coralElbow;
  private final Arm coralWrist;
  private final Arm algaeArm;
  private final Roller algaeIntake;
  private final Elevator elevator;
  private final Roller coralIntake;
  private final CoralCommands coralCommands;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations;
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                // new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2));
        coralElbow = new Arm(new CoralElbowConfig());
        coralWrist = new Arm(new CoralWristConfig());
        elevator = new Elevator(new ElevatorSpecificConfig());
        coralIntake = new Roller(new CoralIntakeConfig());
        algaeArm = new Arm(new AlgaeArmConfig());
        algaeIntake = new Roller(new AlgaeIntakeConfig());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        coralElbow = new Arm(new CoralElbowConfig(false));
        coralWrist = new Arm(new CoralWristConfig(false));
        elevator = new Elevator(new ElevatorSpecificConfig(false));
        coralIntake = new Roller(new CoralIntakeConfig(false));
        algaeArm = new Arm(new AlgaeArmConfig(false));
        algaeIntake = new Roller(new AlgaeIntakeConfig(false));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        coralElbow = new Arm(new ArmConfig() {});
        coralWrist = new Arm(new ArmConfig() {});
        elevator = new Elevator(new ElevatorConfig() {});
        coralIntake = new Roller(new CoralIntakeConfig() {});
        algaeArm = new Arm(new AlgaeArmConfig() {});
        algaeIntake = new Roller(new AlgaeIntakeConfig() {});
        break;
    }

    coralCommands = new CoralCommands(elevator, coralElbow, coralWrist, coralIntake);

    NamedCommands.registerCommand("L1", coralCommands.l1());
    NamedCommands.registerCommand("L2", coralCommands.l2());
    NamedCommands.registerCommand("L3", coralCommands.l3());
    NamedCommands.registerCommand("L4", coralCommands.l4());
    NamedCommands.registerCommand("Coral Station", coralCommands.coralStation());
    NamedCommands.registerCommand("Intake", coralCommands.intake());
    NamedCommands.registerCommand("Release", coralCommands.release());

    SmartDashboard.setDefaultBoolean("Field Oriented", false);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Leave Start", DriveCommands.joystickDrive(drive, () -> 0.2, () -> 0, () -> 0));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    CameraServer.startAutomaticCapture("Front Driver Camera", 0);
    CameraServer.startAutomaticCapture("Coral Camera", 1);

    // Configure the button bindings
    configureButtonBindings();

    // Configure visualizaition
    configureVisualization();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * heightLimitMultiplier(),
            () -> -controller.getLeftX() * heightLimitMultiplier(),
            () -> -controller.getRightX() * heightLimitMultiplier(),
            SmartDashboard.getBoolean("Field Oriented", false)));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Y button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // coral subsystem control
    operatorController.a().whileTrue(coralCommands.coralStation());
    operatorController.b().whileTrue(coralCommands.l2());
    operatorController.x().whileTrue(coralCommands.l3());
    operatorController.y().whileTrue(coralCommands.l4());
    operatorController.start().onTrue(coralCommands.stow());
    operatorController.rightTrigger().whileTrue(coralCommands.intake());
    operatorController.leftTrigger().whileTrue(coralCommands.release());
    DoubleSupplier manualLeft = () -> -operatorController.getLeftY() * 0.5;
    DoubleSupplier manualRight = () -> -operatorController.getRightY() * 0.5;
    operatorController.back().whileTrue(elevator.runPercentCommand(manualRight));
    operatorController.rightBumper().whileTrue(coralElbow.runPercentCommand(manualRight));
    operatorController.leftBumper().whileTrue(coralWrist.runPercentCommand(manualLeft));

    // algae subsystem control
    controller.rightBumper().whileTrue(algaeArm.runPercentCommand(() -> 0.2));
    controller.leftBumper().whileTrue(algaeArm.runPercentCommand(() -> -0.2));
    controller.rightTrigger().whileTrue(algaeIntake.intakeCommand());
    controller.leftTrigger().whileTrue(algaeIntake.releaseCommand());
  }

  private void configureVisualization() {
    Mechanism2d sideView = new Mechanism2d(2, 2);

    MechanismRoot2d elevatorRoot = sideView.getRoot("Elevator Root", 1.2, 0);
    MechanismRoot2d algaeRoot = sideView.getRoot("Algae Root", 1.5, 0);
    elevator.visualization.setAngle(90);
    elevatorRoot.append(elevator.visualization);

    coralElbow.visualization.setLength(.2);
    elevator.visualization.append(coralElbow.visualization);

    coralWrist.visualization.setLength(.2);
    coralElbow.visualization.append(coralWrist.visualization);
    algaeArm.visualization.setLength(.1);
    algaeRoot.append(algaeArm.visualization);

    SmartDashboard.putData("Side View", sideView);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private double heightLimitMultiplier() {
    return 1.0 - (elevator.getHeightPercent() * 0.5);
  }
}
