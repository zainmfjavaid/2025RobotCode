// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ReefAlignCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private PhotonVision photonVision;

    private double maxDriveSpeedFeetPerSecond = Units.metersToFeet(4);
    private double maxRotationalSpeedDegreesPerSecond = Units.radiansToDegrees(4);
    private int targetID;


    public ReefAlignCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        photonVision = new PhotonVision();

        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        targetID = photonVision.getTargetID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distance = photonVision.getDistanceInches();
        double xOffset = photonVision.getXOffsetInches();
        double currentAngleDegrees = swerveSubsystem.getGyroAngle().getDegrees();
        // P only pid control loops to minimize skew (become parallel w apriltag)
        // swerveSubsystem.spin((skewAngleDegrees) * (Constants.DriveConstants.kMaxRotationSpeedRadiansPerSecond * 0.05));
        System.out.println("DISTANCE CALCULATED FROM TARGET: " + distance);
        System.out.println("angle of elevation to target " + photonVision.getPitch());
        System.out.println("xOff: " + xOffset);
        System.out.println("ID & angle: " + targetID + " which is at " + Constants.apriltagAngles[targetID]);
        System.out.println("my angle is " + currentAngleDegrees);
        double xSpeed = (12 - distance) * (maxDriveSpeedFeetPerSecond * .001);
        double ySpeed = (12 - xOffset) * (maxDriveSpeedFeetPerSecond * .001);
        double rotationalSpeed = (Constants.apriltagAngles[targetID] - currentAngleDegrees) * (maxRotationalSpeedDegreesPerSecond * 0.0001);

        //swerveSubsystem.setModuleSpeeds(0, 0, rotationalSpeed);

        photonVision.updateCameraResults();
}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.setModuleSpeeds(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
