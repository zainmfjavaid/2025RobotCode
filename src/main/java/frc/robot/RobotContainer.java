// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.Controller.Button;
import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.WristSubsystem.IntakeState;
import frc.robot.commands.CoralOuttakeCommand;

import frc.robot.hardware.Controller.DriverController;

import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.ElevatorTesting;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
    private final DriverController driverController = new DriverController();
    
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorTesting elevatorTestingSubsystem = new ElevatorTesting();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(swerveSubsystem, driverController));
    // The robot's subsystems and commands are defined here...
    private final Controller controller = new Controller(OperatorConstants.kDriverControllerPort);

    private final CoralIntakeSubsystem coralSubsystem = new CoralIntakeSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    private final WristSubsystem wristSubsystem = new WristSubsystem();
    
    private final DriveCommand driveCommand = new DriveCommand(driveSubsystem, controller);
    private final CoralIntakeCommand coralIntake = new CoralIntakeCommand(coralSubsystem);
    private final CoralOuttakeCommand coralOuttake = new CoralOuttakeCommand(coralSubsystem);
    private final WristCommand groundIntakeCommand = new WristCommand(wristSubsystem, IntakeState.GroundIntake);
    private final WristCommand sourceIntakeCommand = new WristCommand(wristSubsystem, IntakeState.SourceIntake);
    private final WristCommand coralScoreCommand = new WristCommand(wristSubsystem, IntakeState.CoralScore);
    private final WristCommand stowCommand = new WristCommand(wristSubsystem, IntakeState.Stow);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveCommand);
        configureBindings();
    }
    
    private void configureBindings() { 
        // Elevator Testing
        driverController.getButton(DriverController.Button.LB).whileTrue(elevatorTestingSubsystem.goUpCommand());
        driverController.getButton(DriverController.Button.RB).whileTrue(elevatorTestingSubsystem.goDownCommand());

        // Test
        driverController.getButton(DriverController.Button.X).onTrue(new InstantCommand(() -> swerveSubsystem.printEncoderValues()));
        driverController.getButton(DriverController.Button.A).onTrue(new InstantCommand(() -> swerveSubsystem.printGyroValue()));
        driverController.getButton(DriverController.Button.Y).onTrue(new InstantCommand(() -> driverController.printJoystickAxes()));
        driverController.getButton(DriverController.Button.B).onTrue(new InstantCommand(() -> swerveSubsystem.printOdometerPose()));

        // Drive
        driverController.getButton(DriverController.Button.Start).onTrue(new InstantCommand(() -> swerveSubsystem.resetGyroAndOdometer()));
        controller.getButton(Button.Start).onTrue(new InstantCommand(() -> driveSubsystem.reset()));

        // Algae System
        controller.getButton(Button.RB).whileTrue(algaeSubsystem.getIntakeCommand());
        controller.getButton(Button.LB).whileTrue(algaeSubsystem.getOuttakeCommand());
        controller.getButton(Button.A).onTrue(groundIntakeCommand);
        controller.getButton(Button.B).onTrue(sourceIntakeCommand);
        controller.getButton(Button.X).onTrue(coralScoreCommand);
        controller.getButton(Button.Y).onTrue(stowCommand);

        // new JoystickButton(joystick, Button.B2.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(-1)));
        // new JoystickButton(joystick, Button.B3.getPort()).onTrue(new InstantCommand(() -> shooterSubsystem.runShooterAngleMotor(1)));

        controller.getButton(Button.LT).onTrue(coralIntake);    
        controller.getButton(Button.RT).onTrue(coralOuttake);
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoSwerveConstants.kMaxDriveSpeedMetersPerSecond, 
            AutoSwerveConstants.kMaxAccelerationMetersPerSecondSquared);
        trajectoryConfig.setKinematics(swerveSubsystem.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 0),
                new Translation2d(0.5, 0.5),
                new Translation2d(0, -0.5)
            ),
            new Pose2d(0.4, 0.4, Rotation2d.fromDegrees(0)),
            trajectoryConfig
        );

        return new SequentialCommandGroup(
            new AutoDriveCommand(swerveSubsystem, trajectory)
        );
    }
}

