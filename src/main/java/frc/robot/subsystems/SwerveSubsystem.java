package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;

public class SwerveSubsystem {
    private final SwerveModuleKraken frontLeftModule = new SwerveModuleKraken(MotorConstants.kFrontLeftDriveMotorDeviceId, MotorConstants.kFrontLeftAngleMotorDeviceId);
    private final SwerveModuleKraken frontRightModule = new SwerveModuleKraken(MotorConstants.kFrontRightDriveMotorDeviceId, MotorConstants.kFrontRightAngleMotorDeviceId);
    private final SwerveModuleKraken backLeftModule = new SwerveModuleKraken(MotorConstants.kBackLeftDriveMotorDeviceId, MotorConstants.kBackLeftAngleMotorDeviceId);
    private final SwerveModuleKraken backRightModule = new SwerveModuleKraken(MotorConstants.kBackRightDriveMotorDeviceId, MotorConstants.kBackRightAngleMotorDeviceId); 

    private static final Translation2d frontLeftLocation = new Translation2d(RobotConstants.width/2, RobotConstants.length/2);
    private static final Translation2d frontRightLocation = new Translation2d(RobotConstants.width/2, -RobotConstants.length/2);
    private static final Translation2d backLeftLocation = new Translation2d(-RobotConstants.width/2, RobotConstants.length/2);
    private static final Translation2d backRightLocation = new Translation2d(-RobotConstants.width/2, -RobotConstants.length/2);

    // no constructor

    public void swerveDriveAlternative(double ySpeed, double xSpeed, double turnSpeed) {
        double driveAngleRadians = DriveUtils.getAngleRadiansFromComponents(ySpeed, xSpeed);
        double driveSpeed = Math.hypot(xSpeed, ySpeed);
        frontLeftModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        frontRightModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backLeftModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
        backRightModule.setState(driveSpeed, driveAngleRadians, turnSpeed);
    }

}
