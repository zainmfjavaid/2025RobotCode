// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class ReefAlignCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private PhotonVision photonVision;

    private double maxDriveSpeedFeetPerSecond = Units.metersToFeet(SwerveConstants.kMaxWheelDriveSpeedMetersPerSecond);

    public ReefAlignCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        photonVision = new PhotonVision();

        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double skewAngleDegrees = photonVision.getSkew();
        double distance = photonVision.getDistanceInches();
        double xOffset = photonVision.getXOffsetInches();
        // P only pid control loops to minimize skew (become parallel w apriltag)
        swerveSubsystem.spin((skewAngleDegrees) * (SwerveConstants.kMaxRotationSpeedRadiansPerSecond * 0.05));
    
        double xSpeed = (12 - distance) * (maxDriveSpeedFeetPerSecond * 0.01);
        double ySpeed = (12 - xOffset) * (maxDriveSpeedFeetPerSecond * 0.01);

        swerveSubsystem.setModuleSpeeds(xSpeed, ySpeed, 0);
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
