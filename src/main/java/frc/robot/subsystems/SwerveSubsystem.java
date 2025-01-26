package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.hardware.Controller;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

import com.kauailabs.navx.frc.AHRS;

public class SwerveSubsystem extends SubsystemBase {
    private static final Translation2d frontLeftLocation = new Translation2d(RobotConstants.width/2, RobotConstants.length/2);
    private static final Translation2d frontRightLocation = new Translation2d(RobotConstants.width/2, -RobotConstants.length/2);
    private static final Translation2d backLeftLocation = new Translation2d(-RobotConstants.width/2, RobotConstants.length/2);
    private static final Translation2d backRightLocation = new Translation2d(-RobotConstants.width/2, -RobotConstants.length/2);
            
    private final SwerveModuleKraken frontLeftModule = new SwerveModuleKraken(MotorConstants.kFrontLeftDriveMotorDeviceId, MotorConstants.kFrontLeftAngleMotorDeviceId, frontLeftLocation, EncoderConfig.FRONT_LEFT);
    private final SwerveModuleKraken frontRightModule = new SwerveModuleKraken(MotorConstants.kFrontRightDriveMotorDeviceId, MotorConstants.kFrontRightAngleMotorDeviceId, frontRightLocation, EncoderConfig.FRONT_RIGHT);
    private final SwerveModuleKraken backLeftModule = new SwerveModuleKraken(MotorConstants.kBackLeftDriveMotorDeviceId, MotorConstants.kBackLeftAngleMotorDeviceId, backLeftLocation, EncoderConfig.BACK_LEFT);
    private final SwerveModuleKraken backRightModule = new SwerveModuleKraken(MotorConstants.kBackRightDriveMotorDeviceId, MotorConstants.kBackRightAngleMotorDeviceId, backRightLocation, EncoderConfig.BACK_RIGHT); 

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final AHRS gyro = new AHRS(Port.kUSB);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(getKinematics(), new Rotation2d(0), getModulePositions());

    public void setModuleStates(double longitudinalSpeed, double lateralSpeed, double rotationSpeed) {
        frontLeftModule.setState(longitudinalSpeed, lateralSpeed, rotationSpeed);
        frontRightModule.setState(longitudinalSpeed, lateralSpeed, rotationSpeed);
        backLeftModule.setState(longitudinalSpeed, lateralSpeed, rotationSpeed);
        backRightModule.setState(longitudinalSpeed, lateralSpeed, rotationSpeed);
    }

    public void swerveDriveTeleop(Controller controller) {
        double longitudinalSpeed = controller.getLeftStickY() * TeleopSwerveConstants.kMaxDriveSpeedMetersPerSecond;
        double lateralSpeed = controller.getLeftStickX() * TeleopSwerveConstants.kMaxDriveSpeedMetersPerSecond;
        double rotationSpeed = controller.getRightStickX() * TeleopSwerveConstants.kMaxRotationSpeedRadiansPerSecond;
        setModuleStates(longitudinalSpeed, lateralSpeed, rotationSpeed);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};
    }

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetOdometer() {
        odometer.resetPosition(getGyroAngle(), getModulePositions(), new Pose2d(0, 0, getGyroAngle()));
    }

    public void updateOdometer() {
        odometer.update(getGyroAngle(), getModulePositions());
    }
}