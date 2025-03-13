// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorScore extends Command {
    /** Creates a new ElevatorScore. */
    IntakeSubsystem intakeSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    IntakeState intakeState;

    public ElevatorScore(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeState intakeState) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeState = intakeState;

        addRequirements(intakeSubsystem);
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // TODO: NEED TO CHECK THIS STATE
        if (intakeState == IntakeState.TROUGH) {
            //intakeSubsystem.runRollerMotor(-0.5);
        } else {
            //intakeSubsystem.runKickerWheel(0.5);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorSubsystem.resetElevatorPosition();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //intakeSubsystem.runRollerMotor(0);
        //intakeSubsystem.runKickerWheel(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
