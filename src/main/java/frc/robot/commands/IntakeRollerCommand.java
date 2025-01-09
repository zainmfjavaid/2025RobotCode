package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollerCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    private final double duration;
    private double startTimestamp;

    public IntakeRollerCommand(IntakeSubsystem subsystem, double speed, double duration) {
        intakeSubsystem = subsystem;
        this.speed = speed;
        this.duration = duration;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        startTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeRollerMotor(speed);
        intakeSubsystem.setIntakeIndexMotor(speed);
        intakeSubsystem.setIntakeShooterInMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeRollerMotor(0);
        intakeSubsystem.setIntakeIndexMotor(0);
        intakeSubsystem.setIntakeShooterInMotor(0);
    }

    @Override
    // Stops after three seconds
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTimestamp > duration;
    }
}
