// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoncommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SystemSpeeds;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TorchIntakeCommand extends Command {
	/** Creates a new ArmInitCommand. */
	private final IntakeSubsystem intakeSubsystem;
    private int cycles = 0;

	public TorchIntakeCommand(IntakeSubsystem intakeSubsystem) {
		this.intakeSubsystem = intakeSubsystem;

		addRequirements(intakeSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        cycles = 0;
		intakeSubsystem.setGoal(IntakeState.TORCH);
		System.out.println("herehereherehere");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		cycles++;

        if (cycles > 5 && cycles < 75) {
            intakeSubsystem.runRollerMotors(SystemSpeeds.kIntakeRollerSpeed);
        } else if (cycles >= 75 && cycles < 100) {
            intakeSubsystem.setGoal(IntakeState.STOW);
        } else if (cycles >= 100) {
			intakeSubsystem.runRollerMotors(0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intakeSubsystem.setGoal(IntakeState.STOW);
		intakeSubsystem.runRollerMotors(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return cycles >= 101 && intakeSubsystem.atSetpoint(IntakeState.STOW);
	}
}
