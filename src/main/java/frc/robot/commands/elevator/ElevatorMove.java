// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.IntakeConstants.IntakeState;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
    /** Creates a new ElevatorMove. */
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    IntakeState intakeState;

    public ElevatorMove(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeState intakeState) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeState = intakeState;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intakeSubsystem.setGoal(intakeState);
        elevatorSubsystem.setGoal(intakeState);
        elevatorSubsystem.setOverride(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (elevatorSubsystem.atSetpoint()) {
            intakeSubsystem.setGoal(intakeState);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}