// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.CoralIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeCommand extends Command {
  /** Creates a new CorralIntakeCommand. */
  private final CoralIntakeSubsystem coralIntake;
  private final double speed;
  public CoralIntakeCommand(CoralIntakeSubsystem subsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    coralIntake = subsystem;    
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.setSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
