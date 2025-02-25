// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.IntakeState;

import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {

  private final WristSubsystem wristSubsystem;
  private final double armDesiredPosition;
  private final double wristDesiredPosition;

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wristSubsystem, IntakeState intakeState) {
    this.wristSubsystem = wristSubsystem;
    armDesiredPosition = intakeState.armAngle;
    wristDesiredPosition = intakeState.wristAngle;
    addRequirements(wristSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.setWristMotorPostition(wristDesiredPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSubsystem.setArmMotorPosition(armDesiredPosition);
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
