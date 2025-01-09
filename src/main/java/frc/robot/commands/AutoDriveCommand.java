// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoSwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.geometry.Pose2d;

public class AutoDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Timer timer = new Timer();
  private final Trajectory trajectory;

  // private final boolean shuffleboardPID = false;

  // private final ShuffleboardTab tab = Shuffleboard.getTab("Auto Drive PID");
  // private final GenericEntry xP = tab.add("XP", AutoSwerveConstants.kXController.getP()).getEntry();
  // private final GenericEntry yP = tab.add("YP", AutoSwerveConstants.kYController.getP()).getEntry();

  /** Creates a new AutonomousDriveCommand. */
  public AutoDriveCommand(DriveSubsystem subsystem, Trajectory trajectory) {
    driveSubsystem = subsystem;
    this.trajectory = trajectory;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    driveSubsystem.resetOdometer();
    driveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.updateOdometer();

    double currentTime = timer.get();

    State desiredState = trajectory.sample(currentTime); 
    Pose2d desiredPose = desiredState.poseMeters;

    Pose2d currentPose = driveSubsystem.getPose();

    double xError = desiredPose.getX() - currentPose.getX();
    double yError = desiredPose.getY() - currentPose.getY();
    double angleErrorRadians = desiredPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

    SmartDashboard.putNumber("X Error", xError);
    SmartDashboard.putNumber("Y Error", yError);
    SmartDashboard.putNumber("Rotation Error", angleErrorRadians);

    double xSpeed = DriveUtils.normalizeSpeed(AutoSwerveConstants.kXController.calculate(currentPose.getX(), desiredPose.getX()));
    double ySpeed = DriveUtils.normalizeSpeed(AutoSwerveConstants.kYController.calculate(currentPose.getY(), desiredPose.getY()));
    double rotationSpeed = AutoSwerveConstants.kThetaController.calculate(currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

    SmartDashboard.putNumber("X Speed", xSpeed);
    SmartDashboard.putNumber("Y Speed", ySpeed);
    SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

    driveSubsystem.swerveDriveAutoRelativeSpeeds(xSpeed, ySpeed, rotationSpeed);
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
