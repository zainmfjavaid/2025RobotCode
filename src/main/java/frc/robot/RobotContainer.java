// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Constants.AutoSwerveConstants;

import frc.robot.hardware.Controller.DriverController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorTesting;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private ShuffleboardTab configTab = Shuffleboard.getTab("config");
    private SendableChooser<Command> autonChooser = null;
    private final DriverController driverController = new DriverController();
    
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorTesting elevatorTestingSubsystem = new ElevatorTesting();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    

    //private final ReefAlignCommand reefAlignCommand = new ReefAlignCommand(swerveSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(swerveSubsystem, driverController));
        configureBindings();
    
        autonChooser = AutoBuilder.buildAutoChooser("");
        NamedCommands.registerCommand("elevatorUp", elevatorTestingSubsystem.goUpCommand());
        NamedCommands.registerCommand("elevatorDown", elevatorTestingSubsystem.goDownCommand());
        configTab.add("Auton Selection", autonChooser).withSize(3, 1);
    }

    private void configureBindings() { 
        // Elevator Testing
        driverController.getButton(DriverController.Button.LB).whileTrue(elevatorTestingSubsystem.goUpCommand());
        driverController.getButton(DriverController.Button.RB).whileTrue(elevatorTestingSubsystem.goDownCommand());

        // Intake Testing
        driverController.getButton(DriverController.Button.A).onTrue(intakeSubsystem.rotateWristTest());

        // Climb 
        driverController.getButton(DriverController.Button.X).onTrue(new InstantCommand(() -> climbSubsystem.setClimb()));
        // Alignment (uncomment PLEASE)
        //driverController.getButton(DriverController.Button.Y).whileTrue(reefAlignCommand);

        // Test
        // driverController.getButton(DriverController.Button.X).onTrue(new InstantCommand(() -> swerveSubsystem.printEncoderValues()));
        // driverController.getButton(DriverController.Button.A).onTrue(new InstantCommand(() -> swerveSubsystem.printGyroValue()));
        // driverController.getButton(DriverController.Button.B).onTrue(new InstantCommand(() -> swerveSubsystem.printOdometerPose()));

        // Drive
        driverController.getButton(DriverController.Button.Start).onTrue(new InstantCommand(() -> swerveSubsystem.resetGyroAndOdometer()));
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
        return autonChooser.getSelected();
    }
}

