// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SystemSpeeds;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SourceIntakeCommand extends Command {
    /** Creates a new IntakeCommand. */
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public SourceIntakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevatorSubsystem.setGoal(IntakeState.SOURCE);
        elevatorSubsystem.setOverride(false);
        intakeSubsystem.setGoal(IntakeState.SOURCE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (intakeSubsystem.atSetpoint(IntakeState.SOURCE)) {
            intakeSubsystem.runRollerMotors(SystemSpeeds.kIntakeRollerSpeed);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setGoal(IntakeState.STOW);
        intakeSubsystem.setGoal(IntakeState.STOW);
        intakeSubsystem.runRollerMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}