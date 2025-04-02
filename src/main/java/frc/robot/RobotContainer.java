// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.logging.Level;

import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.hardware.Controller.DriverController;
import frc.robot.hardware.Controller.OperatorController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.autoncommands.ArmInitCommand;
import frc.robot.commands.autoncommands.TorchIntakeCommand;
import frc.robot.commands.autoncommands.AutonTroughScoreCommand;
import frc.robot.commands.autoncommands.LevelFourUp;
import frc.robot.commands.autoncommands.LevelFourUpAndDown;
import frc.robot.commands.elevator.ArmHook;
import frc.robot.commands.elevator.ElevatorMove;
import frc.robot.commands.elevator.ElevatorScore;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LeftReefAlignCommand;
import frc.robot.commands.RightReefAlignCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    private final SendableChooser<Command> autonChooser;

    // Controllers
    private final DriverController driverController = new DriverController();
    private final OperatorController operatorController = new OperatorController();
    
    // Subsystems
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    // Elevator Commands
    private final ElevatorMove levelOneCommand = new ElevatorMove(intakeSubsystem, elevatorSubsystem, IntakeState.TROUGH);
    private final ElevatorMove levelTwoCommand = new ElevatorMove(intakeSubsystem, elevatorSubsystem, IntakeState.L2);
    private final ElevatorMove levelThreeCommand = new ElevatorMove(intakeSubsystem, elevatorSubsystem, IntakeState.L3);
    private final ElevatorMove levelFourCommand = new ElevatorMove(intakeSubsystem, elevatorSubsystem, IntakeState.L4);

    private final ElevatorScore elevatorScoreCommand = new ElevatorScore(intakeSubsystem, elevatorSubsystem, IntakeState.L4); // create new cmd AT the trigger
    private final ArmHook armHookCommand = new ArmHook(intakeSubsystem, elevatorSubsystem, IntakeState.L4);

    private final LeftReefAlignCommand leftReefAlignCommand = new LeftReefAlignCommand();
    private final RightReefAlignCommand rightReefAlignCommand = new RightReefAlignCommand();

    // Intake Commands
    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, elevatorSubsystem);
    private final SourceIntakeCommand sourceIntakeCommand = new SourceIntakeCommand(intakeSubsystem, elevatorSubsystem);

    // Auton Named Commands
    private final ArmInitCommand armInitCommand = new ArmInitCommand(intakeSubsystem);
    private final TorchIntakeCommand torchIntakeCommand = new TorchIntakeCommand(intakeSubsystem);
    private final AutonTroughScoreCommand troughScoreCommand = new AutonTroughScoreCommand(intakeSubsystem);
    private final LevelFourUpAndDown levelFourUpAndDown = new LevelFourUpAndDown(elevatorSubsystem, intakeSubsystem);
    private final LevelFourUp levelFourUp = new LevelFourUp(elevatorSubsystem, intakeSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("InitializeArm", armInitCommand);
        NamedCommands.registerCommand("TorchIntake", torchIntakeCommand);
        NamedCommands.registerCommand("TroughScore", troughScoreCommand);
        NamedCommands.registerCommand("FullLevelFour", levelFourUpAndDown);
        NamedCommands.registerCommand("LeftAlign", leftReefAlignCommand);
        NamedCommands.registerCommand("RightAlign", rightReefAlignCommand);

        swerveSubsystem.setDefaultCommand(swerveSubsystem.getCustomDriveCommand(driverController::getLeftStickY, driverController::getLeftStickX, driverController::getRightStickX));

        configureBindings();
    
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autonChooser);
    }
    
    private void configureBindings() { 
        // Driver Controls
        driverController.getButton(DriverController.Button.LB).whileTrue(intakeCommand);
        driverController.getButton(DriverController.Button.Y).whileTrue(sourceIntakeCommand);
        driverController.getButton(DriverController.Button.X).whileTrue(intakeSubsystem.outtakeCommand());

        driverController.getButton(DriverController.Button.RB).onTrue(elevatorScoreCommand);
        // driverController.getButton(DriverController.Button.RB).onTrue(armHookCommand);
        driverController.getButton(DriverController.Button.Back).onTrue(new InstantCommand(swerveSubsystem::resetGyro));

        // some testing cmds
        operatorController.getButton(OperatorController.Button.RT).whileTrue(rightReefAlignCommand);
        operatorController.getButton(OperatorController.Button.LT).whileTrue(leftReefAlignCommand);

        // Operator Controls
        operatorController.getButton(OperatorController.Button.A).onTrue(levelOneCommand);
        operatorController.getButton(OperatorController.Button.B).onTrue(levelTwoCommand);
        operatorController.getButton(OperatorController.Button.Y).onTrue(levelThreeCommand);
        operatorController.getButton(OperatorController.Button.X).onTrue(levelFourCommand);

        operatorController.getButton(OperatorController.Button.RB).whileTrue(climbSubsystem.climbCommandTest());
        operatorController.getButton(OperatorController.Button.LB).whileTrue(climbSubsystem.reverseClimbCommandTest());

        operatorController.getButton(OperatorController.Button.Back).onTrue(armInitCommand);
        operatorController.getButton(OperatorController.Button.Start).onTrue(torchIntakeCommand);
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("New New Auto"); 
    }
}

