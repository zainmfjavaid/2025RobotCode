// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.geometry.Pose2d;

public class AutoDriveCommand extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final Timer timer = new Timer();
  private final Trajectory trajectory;

  /** Creates a new AutonomousDriveCommand. */
  public AutoDriveCommand(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
    this.swerveSubsystem = swerveSubsystem;
    this.trajectory = trajectory;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    swerveSubsystem.resetOdometer();
    swerveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsystem.updateOdometer();

    double currentTime = timer.get();

    State desiredState = trajectory.sample(currentTime); 
    Pose2d desiredPose = desiredState.poseMeters;

    Pose2d currentPose = swerveSubsystem.getPose();

    double longitudinalSpeed = AutoSwerveConstants.kXController.calculate(currentPose.getX(), desiredPose.getX());
    double lateralSpeed = AutoSwerveConstants.kYController.calculate(currentPose.getY(), desiredPose.getY());
    double rotationSpeed = AutoSwerveConstants.kThetaController.calculate(currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

    swerveSubsystem.setModuleSpeeds(longitudinalSpeed, lateralSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
