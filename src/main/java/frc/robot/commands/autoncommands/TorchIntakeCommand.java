// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoncommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TorchIntakeCommand extends Command {
	/** Creates a new ArmInitCommand. */
	IntakeSubsystem intakeSubsystem;
    int cycles = 0;

	public TorchIntakeCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;

		addRequirements(intakeSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        cycles = 0;
        System.out.println("RUNNING TORCH");
		intakeSubsystem.setGoal(IntakeState.TORCH);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		cycles++;
        System.out.println(cycles);

        if (cycles > 5 && cycles < 50) {
            intakeSubsystem.runRollerMotors(0.8);
        } else if (cycles >= 50) {
            System.out.println("DONEDONEDONE");
            intakeSubsystem.runRollerMotors(0);
            intakeSubsystem.setGoal(IntakeState.STOW);
        }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        System.out.println("ENDING");
    }

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return cycles >= 50 && intakeSubsystem.atSetpoint(IntakeState.STOW);
	}
}
