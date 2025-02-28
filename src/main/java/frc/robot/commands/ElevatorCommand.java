// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  /** Creates a new ElevatorCommand. */
  ElevatorSubsystem elevatorSubsystem;
  IntakeState intakeState;
  IntakeSubsystem intakeSubsystem;

  public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, IntakeState position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeState = position;
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(elevatorSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setPosition(intakeState);
    intakeSubsystem.setIntakeState(intakeState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (intakeState.getElevatorValue() == 0) {
      intakeSubsystem.runRollerMotor(0.5);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // WRONG
    return elevatorSubsystem.atSetpoint(intakeState);
  }
}
