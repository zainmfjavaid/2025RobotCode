// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.hardware.Controller.DriverController;
import frc.robot.hardware.Controller.OperatorController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.InstantCommand;
 import frc.robot.commands.AutoDriveCommand;
 import frc.robot.commands.ReefAlignCommand;
 import frc.robot.commands.SourceIntakeCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.autoncommands.ArmInitCommand;
import frc.robot.commands.autoncommands.TorchIntakeCommand;
import frc.robot.commands.autoncommands.TroughScoreCommand;
import frc.robot.commands.elevator.ElevatorScore;
import frc.robot.commands.elevator.L1;
import frc.robot.commands.elevator.L2;
import frc.robot.commands.elevator.L3;
import frc.robot.commands.elevator.L4;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    //private ShuffleboardTab configTab = Shuffleboard.getTab("config");

    //private final SendableChooser<Command> autonChooser;
    private final DriverController driverController = new DriverController();
    private final OperatorController operatorController = new OperatorController();
    
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    AutoDriveCommand autoDriveCommand = new AutoDriveCommand(swerveSubsystem);

    private final SendableChooser<Command> autoChooser;
    // Elevator commands

    // Future fix: this seems inefficient, maybe create a single command class instead having 4 different command classes
    private final L1 levelOneCommand = new L1(intakeSubsystem, elevatorSubsystem);
    private final L2 levelTwoCommand = new L2(intakeSubsystem, elevatorSubsystem);
    private final L3 levelThreeCommand = new L3(intakeSubsystem, elevatorSubsystem);
    private final L4 LevelFourCommand = new L4(intakeSubsystem, elevatorSubsystem);
    private final SourceIntakeCommand sourceIntakeCommand = new SourceIntakeCommand(intakeSubsystem, elevatorSubsystem);

    private final ElevatorScore elevatorScoreCommand = new ElevatorScore(intakeSubsystem, elevatorSubsystem, IntakeState.L4); // create new cmd AT the trigger

    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, elevatorSubsystem);
    
    //private final ElevatorTesting elevatorTestingSubsystem = new ElevatorTesting();
    // private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private final ReefAlignCommand reefAlignCommand = new ReefAlignCommand(swerveSubsystem);

    private final ArmInitCommand armInitCommand = new ArmInitCommand(intakeSubsystem);
    private final TorchIntakeCommand torchIntakeCommand = new TorchIntakeCommand(intakeSubsystem);
    private final TroughScoreCommand troughScoreCommand = new TroughScoreCommand(intakeSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("InitializeArm", armInitCommand);
        NamedCommands.registerCommand("TorchIntake", torchIntakeCommand);
        NamedCommands.registerCommand("TroughScore", troughScoreCommand);

        swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(swerveSubsystem, driverController));
    
        configureBindings();
    
        //autonChooser = AutoBuilder.buildAutoChooser();

        //NamedCommands.registerCommand("elevatorUp", elevatorTestingSubsystem.goUpCommand());
        //NamedCommands.registerCommand("elevatorDown", elevatorTestingSubsystem.goDownCommand());
        //configTab.add("Auton Selection", autonChooser).withSize(3, 1);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() { 
        // Driver controls
        driverController.getButton(DriverController.Button.RB).onTrue(elevatorScoreCommand);
        driverController.getButton(DriverController.Button.LB).whileTrue(intakeCommand);
        driverController.getButton(DriverController.Button.Y).whileTrue(sourceIntakeCommand);

        driverController.getButton(DriverController.Button.X).whileTrue(new StartEndCommand(() -> intakeSubsystem.runRollerMotors(-0.6), () -> intakeSubsystem.runRollerMotors(0), intakeSubsystem));
        driverController.getButton(DriverController.Button.Back).onTrue(new InstantCommand(() -> swerveSubsystem.resetGyroAndOdometer()));

        // Operator Controls
        //operatorController.getButton(OperatorController.Button.RB).whileTrue(new StartEndCommand(elevatorSubsystem::goUp, elevatorSubsystem::stop, elevatorSubsystem));
        //operatorController.getButton(OperatorController.Button.LB).whileTrue(new StartEndCommand(elevatorSubsystem::goDown, elevatorSubsystem::stop, elevatorSubsystem));

        operatorController.getButton(OperatorController.Button.A).onTrue(levelOneCommand);
        operatorController.getButton(OperatorController.Button.B).onTrue(levelTwoCommand);
        operatorController.getButton(OperatorController.Button.Y).onTrue(levelThreeCommand);
        operatorController.getButton(OperatorController.Button.X).onTrue(LevelFourCommand);


        //driverController.getButton(DriverController.Button.LB).whileTrue(elevatorTestingSubsystem.goDownCommand());
        // driverController.getButton(DriverController.Button.RB).whileTrue(elevatorTestingSubsystem.goUpCommand());

        // operatorController.getButton(OperatorController.Button.RB).whileTrue(climbSubsystem.climbCommandTest());
        // operatorController.getButton(OperatorController.Button.LB).whileTrue(climbSubsystem.reverseClimbCommandTest());
        //driverController.getButton(DriverController.Button.Y).onTrue(new InstantCommand(swerveSubsystem::toggleSpeedConstant, swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        //     AutoSwerveConstants.kMaxDriveSpeedMetersPerSecond, 
        //     AutoSwerveConstants.kMaxAccelerationMetersPerSecondSquared);
        // trajectoryConfig.setKinematics(swerveSubsystem.getKinematics());

        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     List.of(
        //         new Translation2d(1, 0),
        //         new Translation2d(0.5, 0.5),
        //         new Translation2d(0, -0.5)
        //     ),
        //     new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(0)),
        //     trajectoryConfig
        // );

        // return new SequentialCommandGroup(
        //     new AutoDriveCommand(swerveSubsystem, trajectory)
        // );
        //return autonChooser.getSelected();

        return autoChooser.getSelected();
        // return autoDriveCommand;
        
    }
}

