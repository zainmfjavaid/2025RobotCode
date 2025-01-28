// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Controller.DriverController;
import frc.robot.Controller.OperatorController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriverController driverController = new DriverController();
    private final OperatorController operatorController = new OperatorController();

    private final CoralIntakeSubsystem coralSubsystem = new CoralIntakeSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

    private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, driverController);

    private final CoralIntakeCommand coralIntake = new CoralIntakeCommand(coralSubsystem);
    private final CoralOuttakeCommand coralOuttake = new CoralOuttakeCommand(coralSubsystem);

    private final WristCommand groundIntakeCommand = new WristCommand(wristSubsystem, IntakeState.GroundIntake);
    private final WristCommand sourceIntakeCommand = new WristCommand(wristSubsystem, IntakeState.SourceIntake);
    private final WristCommand coralScoreCommand = new WristCommand(wristSubsystem, IntakeState.CoralScore);
    private final WristCommand stowCommand = new WristCommand(wristSubsystem, IntakeState.Stow);

    private final ElevatorCommand elevatorGroundCommand = new ElevatorCommand(elevatorSubsystem, ElevatorPosition.Ground);
    private final ElevatorCommand elevatorL1Command = new ElevatorCommand(elevatorSubsystem, ElevatorPosition.L1);
    private final ElevatorCommand elevatorL2Command = new ElevatorCommand(elevatorSubsystem, ElevatorPosition.L2);
    private final ElevatorCommand elevatorL3Command = new ElevatorCommand(elevatorSubsystem, ElevatorPosition.L3);
    private final ElevatorCommand elevatorL4Command = new ElevatorCommand(elevatorSubsystem, ElevatorPosition.L4);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveCommand);
        configureBindings();
    }
    
    private void configureBindings() { 
        // Reset drive gyro and odometer
        driverController.getButton(DriverController.Button.Start).onTrue(new InstantCommand(() -> driveSubsystem.reset()));

        // Set wrist
        driverController.getButton(DriverController.Button.A).onTrue(groundIntakeCommand);
        driverController.getButton(DriverController.Button.B).onTrue(sourceIntakeCommand);
        driverController.getButton(DriverController.Button.X).onTrue(coralScoreCommand);
        driverController.getButton(DriverController.Button.Y).onTrue(stowCommand);

        // Coral Intake
        driverController.getButton(DriverController.Button.RB).onTrue(coralIntake);    
        driverController.getButton(DriverController.Button.RT).onTrue(coralOuttake);    

        // Elevator Stages
        operatorController.getButton(OperatorController.Button.Start).onTrue(elevatorGroundCommand);
        operatorController.getButton(OperatorController.Button.X).onTrue(elevatorL1Command);
        operatorController.getButton(OperatorController.Button.Y).onTrue(elevatorL2Command);
        operatorController.getButton(OperatorController.Button.A).onTrue(elevatorL3Command);
        operatorController.getButton(OperatorController.Button.B).onTrue(elevatorL4Command);
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoSwerveConstants.kMaxDriveSpeedMetersPerSecond, 
            AutoSwerveConstants.kMaxAccelerationMetersPerSecondSquared);
        trajectoryConfig.setKinematics(driveSubsystem.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                // new Translation2d(1, 0),
                // new Translation2d(0.5, 0.5),
                // new Translation2d(0, -0.5)
            ),
            new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(0)),
            trajectoryConfig
        );

        return new SequentialCommandGroup(
            // Drive
            new AutoDriveCommand(driveSubsystem, trajectory)
            // Intake
            // new IntakeDeployCommand(intakeSubsystem, true),
            // new IntakeRollerCommand(intakeSubsystem, 0.3, 0.5),
            // new IntakeDeployCommand(intakeSubsystem, false)
        );
    }
}
