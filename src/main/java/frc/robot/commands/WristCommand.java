// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.WristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristCommand extends Command {

  private final WristSubsystem wristSubsystem;
  private final double armDesiredPosition;
  private final double wristDesiredPosition;

  public enum IntakeState {
    GroundIntake(0), 
    SourceIntake(45), 
    CoralScore(45), 
    Stow(90);

    public final double degrees;

    private IntakeState(double degrees) {
      this.degrees = degrees;
    }
  } 

  /** Creates a new WristCommand. */
  public WristCommand(WristSubsystem wristSubsystem, double armAngle, IntakeState intakeState) {
    this.wristSubsystem = wristSubsystem;
    armDesiredPosition = armAngle;
    wristDesiredPosition = intakeState.degrees;
    addRequirements(wristSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristSubsystem.setArmMotorPosition(armDesiredPosition);
    wristSubsystem.setWristMotorPostition(wristDesiredPosition);
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
