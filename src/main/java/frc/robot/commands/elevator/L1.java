// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L1 extends Command {
    /** Creates a new L1. */
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    SwerveSubsystem swerveSubsystem;

    public L1(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorSubsystem.setGoal(IntakeState.TROUGH);
        elevatorSubsystem.setOverride(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (elevatorSubsystem.atSetpoint()) {
            intakeSubsystem.setGoal(IntakeState.TROUGH);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: ADD END CONDITION. CHECK IF ELEVATOR AT POSITION AND INTAKE AT POSITION
        return false;
    }
}