// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SystemSpeeds;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.hardware.Controller.DriverController;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
    /** Creates a new IntakeCommand. */
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final DriverController driverController = new DriverController();

    public IntakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSubsystem.setGoal(IntakeState.INTAKE);
        elevatorSubsystem.setGoal(IntakeState.INTAKE);
        elevatorSubsystem.setOverride(false);
        SwerveSubsystem.isFieldRelative = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intakeSubsystem.atSetpoint(IntakeState.INTAKE)) {
            intakeSubsystem.runRollerMotors(SystemSpeeds.kIntakeRollerSpeed);
        }
        if (intakeSubsystem.intakeHasCoral()) {
            driverController.activateRumble();
        } else {
            driverController.deactivateRumble();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.runRollerMotors(0);
        SwerveSubsystem.isFieldRelative = true;
        if (intakeSubsystem.intakeHasCoral()) {
            elevatorSubsystem.setGoal(IntakeState.PASSIVERAISE);
            intakeSubsystem.setGoal(IntakeState.PASSIVERAISE);
        } else {
            intakeSubsystem.setGoal(IntakeState.STOW);
        }
        driverController.deactivateRumble();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
