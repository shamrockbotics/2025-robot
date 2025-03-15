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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.roller.*;
import frc.robot.subsystems.vision.*;
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
  private final Elevator elevator;
  private final Roller coralIntake;
  private final Roller algaeIntake;
  private final Arm climber;

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
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2));
        coralElbow = new Arm(new CoralElbowConfig());
        coralWrist = new Arm(new CoralWristConfig());
        algaeArm = new Arm(new AlgaeArmConfig());
        elevator = new Elevator(new ElevatorSpecificConfig());
        coralIntake = new Roller(new CoralIntakeConfig());
        algaeIntake = new Roller(new AlgaeIntakeConfig());
        climber = new Arm(new ClimberConfig());
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
        algaeArm = new Arm(new AlgaeArmConfig(false));
        elevator = new Elevator(new ElevatorSpecificConfig(false));
        coralIntake = new Roller(new CoralIntakeConfig(false));
        algaeIntake = new Roller(new AlgaeIntakeConfig(false));
        climber = new Arm(new ClimberConfig(false));
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
        algaeArm = new Arm(new ArmConfig() {});
        elevator = new Elevator(new ElevatorConfig() {});
        coralIntake = new Roller(new CoralIntakeConfig() {});
        algaeIntake = new Roller(new AlgaeIntakeConfig() {});
        climber = new Arm(new ClimberConfig() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    UsbCamera climberCamera = CameraServer.startAutomaticCapture(0);
    UsbCamera coralCamera = CameraServer.startAutomaticCapture(1);

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Default arm command, hold in position
    coralElbow.setDefaultCommand(
        Commands.run(
            () -> {
              coralElbow.hold();
            },
            coralElbow));
    coralWrist.setDefaultCommand(
        Commands.run(
            () -> {
              coralWrist.hold();
            },
            coralWrist));
    algaeArm.setDefaultCommand(
        Commands.run(
            () -> {
              algaeArm.hold();
            },
            algaeArm));
    elevator.setDefaultCommand(
        Commands.run(
            () -> {
              elevator.hold();
            },
            elevator));
    climber.setDefaultCommand(
        Commands.run(
            () -> {
              climber.stop();
            },
            climber));
    coralIntake.setDefaultCommand(
        Commands.run(
            () -> {
              coralIntake.stop();
            },
            coralIntake));
    algaeIntake.setDefaultCommand(
        Commands.run(
            () -> {
              algaeIntake.stop();
            },
            algaeIntake));

    // loading station
    operatorController
        .a()
        .whileTrue(
            Commands.run(
                () -> {
                  elevator.runToHeight(0.13);
                  coralElbow.runToAngle(-0.7);
                  coralWrist.runToAngle(0.1);
                },
                elevator,
                coralElbow,
                coralWrist));

    // level 2
    operatorController
        .b()
        .whileTrue(
            Commands.run(
                () -> {
                  elevator.runToHeight(.81);
                  coralElbow.runToAngle(-.79);
                  coralWrist.runToAngle(-1.4);
                },
                elevator,
                coralElbow,
                coralWrist));

    // level 3
    operatorController
        .x()
        .whileTrue(
            Commands.run(
                () -> {
                  elevator.runToHeight(.988);
                  coralElbow.runToAngle(-0.79); // -0.458 radians
                  coralWrist.runToAngle(-1.4); // -1.513 radians
                },
                elevator,
                coralElbow,
                coralWrist));

    // level 4
    operatorController
        .y()
        .whileTrue(
            Commands.run(
                () -> {
                  elevator.runToHeight(1.29);
                  coralElbow.runToAngle(0.0);
                  coralWrist.runToAngle(-1.3);
                },
                elevator,
                coralElbow,
                coralWrist));

    operatorController
        .rightStick()
        .whileTrue(
            Commands.run(
                () -> {
                  climber.runToAngle(1.4);
                },
                climber));

    operatorController
        .leftStick()
        .whileTrue(
            Commands.run(
                () -> {
                  climber.runToAngle(0);
                },
                climber));

    // stow position
    operatorController
        .start()
        .whileTrue(
            Commands.run(
                () -> {
                  elevator.runToHeight(0);
                  coralElbow.runToAngle(0);
                  coralWrist.runToAngle(-0.6);
                },
                elevator,
                coralElbow,
                coralWrist));

    // coral arm manual control
    operatorController
        .rightBumper()
        .whileTrue(
            Commands.run(
                () -> {
                  coralElbow.run(-operatorController.getRightY() * 0.5);
                },
                coralElbow));
    operatorController
        .leftBumper()
        .whileTrue(
            Commands.run(
                () -> {
                  coralWrist.run(-operatorController.getLeftY() * 0.5);
                },
                coralWrist));

    operatorController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  coralIntake.run(0.60);
                },
                coralIntake));
    operatorController
        .leftTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  coralIntake.run(-1.0);
                },
                coralIntake));
  }

  private void configureVisualization() {
    Mechanism2d sideView = new Mechanism2d(2, 2);

    MechanismRoot2d elevatorRoot = sideView.getRoot("Elevator Root", 1.2, 0);

    elevator.visualization.setAngle(90);
    elevatorRoot.append(elevator.visualization);

    coralElbow.visualization.setLength(.2);
    elevator.visualization.append(coralElbow.visualization);

    coralWrist.visualization.setLength(.2);
    coralElbow.visualization.append(coralWrist.visualization);

    MechanismRoot2d algaeRoot = sideView.getRoot("Algae Root", 0.8, 0);
    MechanismRoot2d climberRoot = sideView.getRoot("Climber Root", 1.5, 0);
    climber.visualization.setLength(.3);
    climberRoot.append(climber.visualization);
    algaeArm.visualization.setLength(.2);
    algaeArm.visualizationAngleOffset = () -> 90;
    algaeArm.visualizationReversed = true;
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
}
