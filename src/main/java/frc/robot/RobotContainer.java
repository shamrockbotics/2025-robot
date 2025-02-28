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
import edu.wpi.first.math.controller.PIDController;
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
  private final Arm jellybeanArm;
  private final Elevator elevator;

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
        jellybeanArm = new Arm(new JellybeanArmConfig());
        elevator = new Elevator(new ElevatorSpecificConfig());
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
        jellybeanArm = new Arm(new JellybeanArmConfig(false));
        elevator = new Elevator(new ElevatorSpecificConfig(false));
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
        jellybeanArm = new Arm(new ArmConfig() {});
        elevator = new Elevator(new ElevatorConfig() {});
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

    // Auto aim command example
    @SuppressWarnings("resource")
    PIDController aimController = new PIDController(0.2, 0.0, 0.0);
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    controller
        .y()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      aimController.reset();
                    },
                    drive)
                .andThen(
                    DriveCommands.joystickDrive(
                        drive,
                        () -> 0.0,
                        () -> 0.0,
                        () -> aimController.calculate(vision.getTargetX(0).getRadians()))));

    // Default arm command, hold in position
    jellybeanArm.setDefaultCommand(
        Commands.run(
            () -> {
              jellybeanArm.hold();
            },
            jellybeanArm));

    operatorController
        .a()
        .whileTrue(
            Commands.run(
                () -> {
                  jellybeanArm.run(0.1);
                },
                jellybeanArm));

    operatorController
        .b()
        .whileTrue(
            Commands.run(
                () -> {
                  jellybeanArm.run(-0.1);
                },
                jellybeanArm));

    operatorController
        .x()
        .whileTrue(
            Commands.run(
                () -> {
                  jellybeanArm.runToAngle(1.4);
                },
                jellybeanArm));

    operatorController
        .y()
        .whileTrue(
            Commands.run(
                () -> {
                  jellybeanArm.runToAngle(0);
                },
                jellybeanArm));

    operatorController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  elevator.runToAngle(0);
                },
                elevator));
  }

  private void configureVisualization() {
    Mechanism2d jellybeanView = new Mechanism2d(60, 60);
    Mechanism2d elevatorView = new Mechanism2d(60, 60);

    MechanismRoot2d elevatorRoot = elevatorView.getRoot("Elevator Root", 30, 20);
    MechanismLigament2d elevatorHolder = new MechanismLigament2d("Elevator Holder", 20, 55);

    MechanismRoot2d armRoot = jellybeanView.getRoot("Arm Root", 30, 20);
    MechanismLigament2d armHolder = new MechanismLigament2d("Arm Holder", 20, 55);
    armRoot.append(armHolder);
    elevatorRoot.append(elevatorHolder);
    elevator.visualization.setLength(20);
    elevator.visualizationAngleOffset = () -> elevatorHolder.getLength();
    elevatorHolder.append(elevator.visualization);

    jellybeanArm.visualization.setLength(20);
    jellybeanArm.visualizationAngleOffset = () -> armHolder.getAngle();
    armHolder.append(jellybeanArm.visualization);

    SmartDashboard.putData("Jellybean", jellybeanView);
    SmartDashboard.putData("Elevator", elevatorView);
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
