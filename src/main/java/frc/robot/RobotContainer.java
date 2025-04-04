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
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.mechanism.*;
import frc.robot.subsystems.roller.*;
import frc.robot.subsystems.vision.*;
import frc.robot.subsystems.winch.*;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

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
  private final Mechanism coralElbow;
  private final Mechanism coralWrist;
  private final Mechanism elevator;
  private final Roller coralIntake;
  private final Mechanism climberArm;
  private final Winch climberWinch;

  // Command factories
  private final CoralCommands coralCommands;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedNetworkBoolean fieldOriented;

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
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2));
        coralElbow = new Mechanism(new CoralElbowConfig());
        coralWrist = new Mechanism(new CoralWristConfig());
        elevator = new Mechanism(new ElevatorConfig());
        coralIntake = new Roller(new CoralIntakeConfig());
        climberArm = new Mechanism(new ClimberArmConfig());
        climberWinch = new Winch(new ClimberWinchConfig());
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
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose));
        coralElbow = new Mechanism(new CoralElbowConfig(false));
        coralWrist = new Mechanism(new CoralWristConfig(false));
        elevator = new Mechanism(new ElevatorConfig(false));
        coralIntake = new Roller(new CoralIntakeConfig(false));
        climberArm = new Mechanism(new ClimberArmConfig(false));
        climberWinch = new Winch(new ClimberWinchConfig(false));
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
        coralElbow = new Mechanism(new MechanismConfig() {});
        coralWrist = new Mechanism(new MechanismConfig() {});
        elevator = new Mechanism(new MechanismConfig() {});
        coralIntake = new Roller(new RollerConfig() {});
        climberArm = new Mechanism(new MechanismConfig() {});
        climberWinch = new Winch(new WinchConfig() {});
        break;
    }
    coralWrist.setPositionOffset(() -> coralElbow.getPosition());

    coralCommands = new CoralCommands(elevator, coralElbow, coralWrist, coralIntake);

    NamedCommands.registerCommand("L1", coralCommands.l1());
    NamedCommands.registerCommand("L2", coralCommands.l2());
    NamedCommands.registerCommand("L3", coralCommands.l3());
    NamedCommands.registerCommand("L4", coralCommands.l4Auto());
    NamedCommands.registerCommand("Coral Station", coralCommands.coralStation());
    NamedCommands.registerCommand("Intake", coralCommands.intake());
    NamedCommands.registerCommand("Release", coralCommands.release());
    NamedCommands.registerCommand("Stow", coralCommands.release());

    fieldOriented = new LoggedNetworkBoolean("/SmartDashboard/Field Oriented", true);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // drive.addSysIdCommands(autoChooser);
    // coralElbow.addSysIdCommands(autoChooser);
    // coralWrist.addSysIdCommands(autoChooser);
    // elevator.addSysIdCommands(autoChooser);

    CameraServer.startAutomaticCapture("Coral Camera", 0);

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
            () -> fieldOriented.get()));

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

    // coral subsystem manual control
    DoubleSupplier manualLeft = () -> -operatorController.getLeftY();
    DoubleSupplier manualRight = () -> -operatorController.getRightY();
    operatorController.back().whileTrue(elevator.runPercentCommand(manualRight));
    operatorController.rightBumper().whileTrue(coralElbow.runPercentCommand(manualRight));
    operatorController.leftBumper().whileTrue(coralWrist.runPercentCommand(manualLeft));

    // climber subsystem controls
    controller
        .leftBumper()
        .whileTrue(
            climberArm.runPercentCommand(
                () -> (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 0.5));
    controller
        .rightBumper()
        .whileTrue(
            climberWinch.runPercentCommand(
                () -> (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())));
  }

  /*controller POVButton UpPov = new POVButton(driverController, 90);
    POVButton[] dpad = new POVButton[] (up, upRight, right,downRight),

    public static final int LEFT_HORIZONTAL = 0, //RIGHT_HORIZONTAL = 2,
            LEFT_VERTICAL = 1; //RIGHT_VERTICAL = 3;

    public static final double DEADZONE = 0.075;

    public Controller(int port) {

      stick = new Joystick(port);

      buttons[LB] = new JoystickButton(getStick(), LB);*
    }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void configureVisualization() {
    Mechanism2d sideView = new Mechanism2d(2, 2);

    MechanismRoot2d elevatorRoot = sideView.getRoot("Elevator Root", 1.2, 0);
    elevator.visualization.setAngle(90);
    elevatorRoot.append(elevator.visualization);

    coralElbow.visualization.setLength(.2);
    coralElbow.visualizationAngleOffset = () -> -90;
    elevator.visualization.append(coralElbow.visualization);

    coralWrist.visualization.setLength(.2);
    coralElbow.visualization.append(coralWrist.visualization);

    SmartDashboard.putData("Side View", sideView);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private double heightLimitMultiplier() {
    return 1.0 - (elevator.getPositionPercent() * 0.5);
  }
}
