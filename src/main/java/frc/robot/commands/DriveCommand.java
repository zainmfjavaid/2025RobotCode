// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.DriveSubsystem;

import frc.robot.hardware.Controller;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {

  private final DriveSubsystem driveSubsystem;
  private final Controller controller;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, Controller controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (DriveConstants.driveType) {
      case ARCADE:
        driveSubsystem.arcadeDrive(getLeftStickY(), getRightStickX());
        break;
      case SWERVE:
        driveSubsystem.swerveDriveTeleopRelativeSpeeds(getLeftStickX(), getLeftStickY(), controller.getRightStickX());
        break;
      case SPINDRIVE:
        driveSubsystem.drive();
        break;
      case SPINANGLE:
        driveSubsystem.spin();
        break;
      case ALIGN:
        driveSubsystem.swerveDriveSpeeds(0, 0, 0);
        break;
      default:
        break;
    }
    driveSubsystem.updateOdometer();
  }

  public double getLeftStickX() {
    return controller.getLeftStickX();
  }
  public double getLeftStickY() {
    return controller.getLeftStickY();
  }
  public double getRightStickX() {
    return controller.getRightStickX();
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
