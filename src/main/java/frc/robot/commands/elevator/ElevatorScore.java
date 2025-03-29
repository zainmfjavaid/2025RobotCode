// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SystemSpeeds;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorScore extends Command {
    /** Creates a new ElevatorScore. */
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeState intakeState;

    private int cycles = 0;

    public ElevatorScore(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeState intakeState) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeState = intakeState;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        cycles = 0;
        // TODO: NEED TO CHECK THIS STATE
        if (intakeSubsystem.atSetpoint(IntakeState.TROUGH)) {
            intakeSubsystem.runRollerMotors(SystemSpeeds.kScoreOuttakeRollerSpeed);
        } else {
            elevatorSubsystem.setOverride(true);
            elevatorSubsystem.goDown();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        cycles++;

        if (cycles > 20 && cycles < 30) {
            intakeSubsystem.runRollerMotors(SystemSpeeds.kScoreOuttakeRollerSpeed);
        } else if (cycles >= 30 && cycles < 60) {
            intakeSubsystem.runRollerMotors(0);

            intakeSubsystem.setGoal(IntakeState.SCORESTOW);
            
            elevatorSubsystem.stop();
        } else if (cycles >= 60) {
            elevatorSubsystem.setGoal(IntakeState.SCORESTOW);
            elevatorSubsystem.setOverride(false);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return cycles > 61;
    }
}
