// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.hardware.Controller.DriverController;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LeftReefAlignCommand extends Command {
    private final PhotonVision photonVision;

    private final double maxDriveSpeedFeetPerSecond = Units.metersToFeet(4);
    private final double maxRotationalSpeedDegreesPerSecond = Units.radiansToDegrees(4);
    private final DriverController driverController = new DriverController();
    private int targetID;

    
    private double distance;
    private double xOffset;
    private double currentAngleDegrees;

    public LeftReefAlignCommand() {
        photonVision = new PhotonVision();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        photonVision.updateCameraResults();
        targetID = photonVision.getTargetID();

        distance = photonVision.getPitch();
        xOffset = photonVision.getTargetYaw();
        currentAngleDegrees = photonVision.getGyroAngle();
    }

    public double normalizeAngle(double Angle){
        double normalized = (Angle % 360);
        if (normalized > 180){
          normalized -= 360;
        } else if (normalized < -180){
            normalized += 360;
        }
        return normalized;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("aligned: " + photonVision.alignedToTarget());

        photonVision.updateCameraResults();

        int currentTargetID = photonVision.getTargetID();

        if (currentTargetID == targetID) {
            distance = photonVision.getPitch();
            xOffset = photonVision.getTargetYaw();
            currentAngleDegrees = photonVision.getGyroAngle();
        }

        // P only pid control loops to minimize skew (become parallel w apriltag)
        // swerveSubsystem.spin((skewAngleDegrees) * (Constants.DriveConstants.kMaxRotationSpeedRadiansPerSecond * 0.05));
        double xSpeed = (Constants.leftReefPitch - distance) * (maxDriveSpeedFeetPerSecond * .0017);
        double ySpeed = (Constants.leftReefYaw - xOffset) * (maxDriveSpeedFeetPerSecond * .0019);
        double rotationalSpeed = normalizeAngle(Constants.apriltagAngles[targetID] - currentAngleDegrees) * (maxRotationalSpeedDegreesPerSecond * 0.0003);
        // System.out.println("my curr angle is " + currentAngleDegrees + " but im fr tryna get to " + Constants.apriltagAngles[targetID]);
        System.out.println("YAW: " + photonVision.getTargetYaw() + " PITCH: " + photonVision.getPitch());
        SwerveSubsystem.setSpeeds(-1 * xSpeed, ySpeed, rotationalSpeed);

        if (photonVision.alignedToTarget()){
            driverController.activateRumble();
        } else {
            driverController.deactivateRumble();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.setSpeeds(0, 0, 0);
        driverController.deactivateRumble();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
