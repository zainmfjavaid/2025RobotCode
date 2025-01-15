// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.Joystick;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveCommand extends Command {

  Joystick joystick = new Joystick(0);

  SwerveSubsystem swerveSubsystem;
  /** Creates a new SwerveCommand. */
  public SwerveCommand(SwerveSubsystem subsystem) {
    swerveSubsystem = subsystem;
    addRequirements(swerveSubsystem);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.swerveDrive(getLeftStickY(), getLeftStickX(), getRightStickX()); 
  }

  public double getLeftStickX(){
    return joystick.getRawAxis(0);
  }
   public double getLeftStickY(){
    return joystick.getRawAxis(1);
  }
   public double getRightStickX(){
    return joystick.getRawAxis(2);
  }
   public double getRightStickY(){
    return joystick.getRawAxis(3);
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
