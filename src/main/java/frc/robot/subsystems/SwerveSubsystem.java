package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

public class SwerveSubsystem {
    private static final Translation2d frontLeftLocation = new Translation2d(RobotConstants.width/2, RobotConstants.length/2);
    private static final Translation2d frontRightLocation = new Translation2d(RobotConstants.width/2, -RobotConstants.length/2);
    private static final Translation2d backLeftLocation = new Translation2d(-RobotConstants.width/2, RobotConstants.length/2);
    private static final Translation2d backRightLocation = new Translation2d(-RobotConstants.width/2, -RobotConstants.length/2);
            
    private final SwerveModuleKraken frontLeftModule = new SwerveModuleKraken(MotorConstants.kFrontLeftDriveMotorDeviceId, MotorConstants.kFrontLeftAngleMotorDeviceId, frontLeftLocation, EncoderConfig.FRONT_LEFT);
    private final SwerveModuleKraken frontRightModule = new SwerveModuleKraken(MotorConstants.kFrontRightDriveMotorDeviceId, MotorConstants.kFrontRightAngleMotorDeviceId, frontRightLocation, EncoderConfig.FRONT_RIGHT);
    private final SwerveModuleKraken backLeftModule = new SwerveModuleKraken(MotorConstants.kBackLeftDriveMotorDeviceId, MotorConstants.kBackLeftAngleMotorDeviceId, backLeftLocation, EncoderConfig.BACK_LEFT);
    private final SwerveModuleKraken backRightModule = new SwerveModuleKraken(MotorConstants.kBackRightDriveMotorDeviceId, MotorConstants.kBackRightAngleMotorDeviceId, backRightLocation, EncoderConfig.BACK_RIGHT); 

    public void swerveDriveControllerSpeeds(double longitundalSpeed, double lateralSpeed, double turnSpeed) {
        frontLeftModule.setState(longitundalSpeed, lateralSpeed, turnSpeed);
        frontRightModule.setState(longitundalSpeed, lateralSpeed, turnSpeed);
        backLeftModule.setState(longitundalSpeed, lateralSpeed, turnSpeed);
        backRightModule.setState(longitundalSpeed, lateralSpeed, turnSpeed);
    }

}
