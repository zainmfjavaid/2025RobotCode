// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.AutoSwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSwerveCommand extends Command {
    /** Creates a new AutoDriveCommand. */
    private final SwerveSubsystem swerveSubsystem;
    private final Trajectory trajectory;

    private final Timer timer = new Timer();

    public AutoSwerveCommand(SwerveSubsystem swerveSubsystem, Trajectory trajectory) {
        this.swerveSubsystem = swerveSubsystem;
        this.trajectory = trajectory;
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
        swerveSubsystem.resetGyroAndOdometer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSubsystem.updateOdometer();

        double currentTime = timer.get();

        State desiredState = trajectory.sample(currentTime); 
        Pose2d desiredPose = desiredState.poseMeters;

        Pose2d currentPose = swerveSubsystem.getPose();

        displayErrors(currentPose, desiredPose);

        double xSpeed = getXSpeed(currentPose.getX(), desiredPose.getX());
        double ySpeed = getYSpeed(currentPose.getY(), desiredPose.getY());
        double rotationSpeed = getRotationSpeed(currentPose.getRotation(), desiredPose.getRotation());

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Y Speed", ySpeed);
        SmartDashboard.putNumber("Rotation Speed", rotationSpeed);

        swerveSubsystem.robotCentricSwerve(xSpeed, ySpeed, rotationSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Shuffleboard

    public void displayErrors(Pose2d currentPose, Pose2d desiredPose) {
        double xError = desiredPose.getX() - currentPose.getX();
        double yError = desiredPose.getY() - currentPose.getY();
        double angleErrorRadians = desiredPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

        SmartDashboard.putNumber("X Error", xError);
        SmartDashboard.putNumber("Y Error", yError);
        SmartDashboard.putNumber("Rotation Error", angleErrorRadians);
    }

    // Speeds

    public double getXSpeed(double currentX, double desiredX) {
        return AutoSwerveConstants.kXController.calculate(currentX, desiredX);
    }

    public double getYSpeed(double currentY, double desiredY) {
        return AutoSwerveConstants.kYController.calculate(currentY, desiredY);
    }

    public double getRotationSpeed(Rotation2d currentRotation, Rotation2d desiredRotation) {
        double errorRadians = desiredRotation.getRadians() - currentRotation.getRadians();
        return AutoSwerveConstants.kThetaController.calculate(0, errorRadians / Constants.kTau);
    }
}
