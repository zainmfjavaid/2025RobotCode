package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeDeployCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double desiredPosition;

    public IntakeDeployCommand(IntakeSubsystem subsystem, boolean isDeploy) {
        intakeSubsystem = subsystem;
        if (isDeploy) {
            desiredPosition = IntakeConstants.kDeployPosition;
        } else {
            desiredPosition = IntakeConstants.kRetractPosition;
        }
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeDeployMotor(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (desiredPosition - intakeSubsystem.getIntakeDeployRelativePosition()) < 0.5;
    }
}
