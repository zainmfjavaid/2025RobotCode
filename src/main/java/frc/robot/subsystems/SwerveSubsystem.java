package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.hardware.Controller.DriverController;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
    private static final Translation2d frontLeftLocation = new Translation2d(RobotConstants.kWidthMeters/2, RobotConstants.kLengthMeters/2);
    private static final Translation2d frontRightLocation = new Translation2d(RobotConstants.kWidthMeters/2, -RobotConstants.kLengthMeters/2);
    private static final Translation2d backLeftLocation = new Translation2d(-RobotConstants.kWidthMeters/2, RobotConstants.kLengthMeters/2);
    private static final Translation2d backRightLocation = new Translation2d(-RobotConstants.kWidthMeters/2, -RobotConstants.kLengthMeters/2);
            
    private final SwerveModule frontLeftModule = new SwerveModule(MotorConstants.kFrontLeftDriveMotorDeviceId, MotorConstants.kFrontLeftAngleMotorDeviceId, frontLeftLocation, EncoderConfig.FRONT_LEFT);
    private final SwerveModule frontRightModule = new SwerveModule(MotorConstants.kFrontRightDriveMotorDeviceId, MotorConstants.kFrontRightAngleMotorDeviceId, frontRightLocation, EncoderConfig.FRONT_RIGHT);
    private final SwerveModule backLeftModule = new SwerveModule(MotorConstants.kBackLeftDriveMotorDeviceId, MotorConstants.kBackLeftAngleMotorDeviceId, backLeftLocation, EncoderConfig.BACK_LEFT);
    private final SwerveModule backRightModule = new SwerveModule(MotorConstants.kBackRightDriveMotorDeviceId, MotorConstants.kBackRightAngleMotorDeviceId, backRightLocation, EncoderConfig.BACK_RIGHT); 

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    private final Pigeon2 gyro = new Pigeon2(20, "CANivore2158");
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(getKinematics(), new Rotation2d(0), getModulePositions());

    private static RobotConfig config;

    public void setModuleStates(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        frontLeftModule.setState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond);
        frontRightModule.setState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond);
        backLeftModule.setState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond);
        backRightModule.setState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 1000); //TODO Replace with max speed
        frontLeftModule.setState(desiredStates[0]);
        frontRightModule.setState(desiredStates[1]);
        backLeftModule.setState(desiredStates[2]);
        backRightModule.setState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveModuleArray = new SwerveModuleState[4];
        swerveModuleArray[0] = frontLeftModule.getState();
        swerveModuleArray[1] = frontRightModule.getState();
        swerveModuleArray[2] = backLeftModule.getState();
        swerveModuleArray[3] = backRightModule.getState();

        return swerveModuleArray;
    }

    public void fieldCentricSwerve(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        double offsetRadians = -getGyroAngle().getRadians();
        setModuleStates(
            longitudinalSpeedMetersPerSecond * Math.cos(offsetRadians) - lateralSpeedMetersPerSecond * Math.sin(offsetRadians), 
            longitudinalSpeedMetersPerSecond * Math.sin(offsetRadians) + lateralSpeedMetersPerSecond * Math.cos(offsetRadians), 
            rotationSpeedRadiansPerSecond);
    }

    public void swerveDriveTeleop(DriverController driveController) {
        double longitudinalSpeedMetersPerSecond = driveController.getLeftStickY() * TeleopSwerveConstants.kMaxDriveSpeedMetersPerSecond;
        double lateralSpeedMetersPerSecond = driveController.getLeftStickX() * TeleopSwerveConstants.kMaxDriveSpeedMetersPerSecond;
        double rotationSpeedRadiansPerSecond = driveController.getRightStickX() * TeleopSwerveConstants.kMaxRotationSpeedRadiansPerSecond;
        fieldCentricSwerve(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(), 
            frontRightModule.getPosition(), 
            backLeftModule.getPosition(), 
            backRightModule.getPosition()
        };
    }

    // counterclockwise is positive
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetOdometer(Pose2d pose) {
        odometer.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public void resetOdometer() {
        odometer.resetPosition(getGyroAngle(), getModulePositions(), new Pose2d(0, 0, getGyroAngle()));
    }

    public void resetGyroAndOdometer() {
        resetGyro();
        resetOdometer();
    }

    public void updateOdometer() {
        odometer.update(getGyroAngle(), getModulePositions());
    }

    // OTHER
    public void spinDriveMotors(double speed) {
        frontLeftModule.setDriveMotorRelativeSpeed(speed);
        frontRightModule.setDriveMotorRelativeSpeed(speed);
        backLeftModule.setDriveMotorRelativeSpeed(speed);
        backRightModule.setDriveMotorRelativeSpeed(speed);
    }

    public void spinAngleMotors(double speed) {
        frontLeftModule.setAngleMotorRelativeSpeed(speed);
        frontRightModule.setAngleMotorRelativeSpeed(speed);
        backLeftModule.setAngleMotorRelativeSpeed(speed);
        backRightModule.setAngleMotorRelativeSpeed(speed);
    }

    public void driveForward(double longitudinalSpeed) {
        setModuleStates(longitudinalSpeed, 0, 0);
    }

    public void driveLaterally(double lateralSpeed) {
        setModuleStates(0, lateralSpeed, 0);
    }

    public void spin(double speedRadiansPerSecond) {
        setModuleStates(0, 0, speedRadiansPerSecond);
    }

    // print
    public void printEncoderValues() {
        System.out.println("Angle Encoder Positions");
        frontLeftModule.printEncoderPositions("FL");
        frontRightModule.printEncoderPositions("FR");
        backLeftModule.printEncoderPositions("BL");
        backRightModule.printEncoderPositions("BR");
    }

    public void printDriveEncoderValues() {
        System.out.println("Drive Encoder Positions");
        frontLeftModule.printDriveEncoderValue("FL");
        frontRightModule.printDriveEncoderValue("FR");
        backLeftModule.printDriveEncoderValue("BL");
        backRightModule.printDriveEncoderValue("BR");
    }

    public void printGyroValue() {
        System.out.println("Gyro Value: " + getGyroAngle());
    }

    public void printOdometerPose() {
        System.out.println("Odometer Pose: " + getPose());
    }

    public void initAuton(){
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometer, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    }
}