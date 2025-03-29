// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;

import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.hardware.Controller.DriverController;
import frc.robot.hardware.Controller.OperatorController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.ReefAlignCommand;
import frc.robot.commands.ScoreMoveBack;
import frc.robot.commands.SourceIntakeCommand;
// import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.autoncommands.ArmInitCommand;
import frc.robot.commands.autoncommands.TorchIntakeCommand;
import frc.robot.commands.autoncommands.AutonTroughScoreCommand;
import frc.robot.commands.elevator.ElevatorScore;
import frc.robot.commands.elevator.L1;
import frc.robot.commands.elevator.L2;
import frc.robot.commands.elevator.L3;
import frc.robot.commands.elevator.L4;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
// import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


public class RobotContainer {
   
    //private ShuffleboardTab configTab = Shuffleboard.getTab("config");

    //private final SendableChooser<Command> autonChooser;
    //private final DriverController driverController = new DriverController();
    //private final OperatorController operatorController = new OperatorController();
    private final DriverController driverController = new DriverController();
    private final OperatorController operatorController = new OperatorController();
    
    
    // private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();


    // AutoDriveCommand autoDriveCommand = new AutoDriveCommand(swerveSubsystem);

    private final SendableChooser<Command> autoChooser;
    // Elevator commands

    // Future fix: this seems inefficient, maybe create a single command class instead having 4 different command classes
    private final L1 levelOneCommand = new L1(intakeSubsystem, elevatorSubsystem);
    private final L2 levelTwoCommand = new L2(intakeSubsystem, elevatorSubsystem);
    private final L3 levelThreeCommand = new L3(intakeSubsystem, elevatorSubsystem);
    private final L4 levelFourCommand = new L4(intakeSubsystem, elevatorSubsystem);

    private final SourceIntakeCommand sourceIntakeCommand = new SourceIntakeCommand(intakeSubsystem, elevatorSubsystem);
    // private final ScoreMoveBack scoreMoveBack = new ScoreMoveBack(swerveSubsystem);

    private final ElevatorScore elevatorScoreCommand = new ElevatorScore(intakeSubsystem, elevatorSubsystem, IntakeState.L4); // create new cmd AT the trigger

    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, elevatorSubsystem);
    
    // private final ElevatorTesting elevatorTestingSubsystem = new ElevatorTesting();
    // private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    // private final ReefAlignCommand reefAlignCommand = new ReefAlignCommand(swerveSubsystem);

    private final ArmInitCommand armInitCommand = new ArmInitCommand(intakeSubsystem);
    private final TorchIntakeCommand torchIntakeCommand = new TorchIntakeCommand(intakeSubsystem);
    private final AutonTroughScoreCommand troughScoreCommand = new AutonTroughScoreCommand(intakeSubsystem);

      SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getLeftStickY() * -1,
                                                                () -> driverController.getLeftStickX() * -1)
                                                            .withControllerRotationAxis(driverController::getRightStickX)
                                                            .deadband(0.2)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("InitializeArm", armInitCommand);
        NamedCommands.registerCommand("TorchIntake", torchIntakeCommand);
        NamedCommands.registerCommand("TroughScore", troughScoreCommand);

        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        // swerveSubsystem.setDefaultCommand(new TeleopDriveCommand(swerveSubsystem, driverController));
        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

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
        // JoystickButton rbButtonDriver = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        // rbButtonDriver.onTrue(elevatorScoreCommand);

        driverController.getButton(DriverController.Button.LB).whileTrue(intakeCommand);
        // JoystickButton lbButtonDriver = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        // lbButtonDriver.whileTrue(intakeCommand);
        

        driverController.getButton(DriverController.Button.Y).whileTrue(sourceIntakeCommand);
        // JoystickButton yButtonDriver = new JoystickButton(driverController, XboxController.Button.kY.value);
        // yButtonDriver.whileTrue(sourceIntakeCommand);

        driverController.getButton(DriverController.Button.X).whileTrue(new StartEndCommand(() -> intakeSubsystem.runRollerMotors(-0.6), () -> intakeSubsystem.runRollerMotors(0), intakeSubsystem));
        // driverController.getButton(DriverController.Button.Back).onTrue(new InstantCommand(() -> swerveSubsystem.resetGyroAndOdometer()));

        // Operator Controls
        operatorController.getButton(OperatorController.Button.A).onTrue(levelOneCommand);
        // JoystickButton aButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
        // aButton.onTrue(levelOneCommand);
        operatorController.getButton(OperatorController.Button.B).onTrue(levelTwoCommand);
        // JoystickButton bButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
        // bButton.onTrue(levelTwoCommand);
        operatorController.getButton(OperatorController.Button.Y).onTrue(levelThreeCommand);
        // JoystickButton yButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
        // yButton.onTrue(levelThreeCommand);
        operatorController.getButton(OperatorController.Button.X).onTrue(new SequentialCommandGroup(scoreMoveBack, levelFourCommand));

        operatorController.getButton(OperatorController.Button.RB).whileTrue(climbSubsystem.climbCommandTest());
        // JoystickButton rbButton = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);
        // rbButton.whileTrue(climbSubsystem.climbCommandTest());
        //
        operatorController.getButton(OperatorController.Button.LB).whileTrue(climbSubsystem.reverseClimbCommandTest());
        // JoystickButton lbButton = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);
        // lbButton.whileTrue(climbSubsystem.reverseClimbCommandTest());


        operatorController.getButton(OperatorController.Button.LT).onTrue(elevatorScoreCommand);
       

        operatorController.getButton(OperatorController.Button.RT).whileTrue(intakeCommand);
        
        operatorController.getButton(OperatorController.Button.Y).onTrue(new InstantCommand(swerveSubsystem::toggleSpeedConstant, swerveSubsystem));
    
        driverController.getButton(DriverController.Button.A).onTrue(new InstantCommand(() -> swerveSubsystem.printEncoderValues()));
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

