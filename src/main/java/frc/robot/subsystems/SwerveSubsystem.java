package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.Constants.SwerveConstants.Module;
import frc.robot.hardware.Controller.DriverController;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeftModule = new SwerveModule(Module.FRONT_LEFT);
    private final SwerveModule frontRightModule = new SwerveModule(Module.FRONT_RIGHT);
    private final SwerveModule backLeftModule = new SwerveModule(Module.BACK_LEFT);
    private final SwerveModule backRightModule = new SwerveModule(Module.BACK_RIGHT); 

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Module.FRONT_LEFT.getLocation(), Module.FRONT_RIGHT.getLocation(), Module.BACK_LEFT.getLocation(), Module.BACK_RIGHT.getLocation());

    private final Pigeon2 gyro = new Pigeon2(20, "CANivore2158");
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(getKinematics(), new Rotation2d(0), getModulePositions());

    private static RobotConfig config;

    private double speedConstant = 1.0;

    public void toggleSpeedConstant() {
        if (speedConstant == 1.0) {
            speedConstant = 0.25;
        } else {
            speedConstant = 1.0;
        }
    }

    public void setModuleSpeeds(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        frontLeftModule.setSpeeds(longitudinalSpeedMetersPerSecond * speedConstant, lateralSpeedMetersPerSecond * speedConstant, rotationSpeedRadiansPerSecond * speedConstant);
        frontRightModule.setSpeeds(longitudinalSpeedMetersPerSecond * speedConstant, lateralSpeedMetersPerSecond * speedConstant, rotationSpeedRadiansPerSecond * speedConstant);
        backLeftModule.setSpeeds(longitudinalSpeedMetersPerSecond * speedConstant, lateralSpeedMetersPerSecond * speedConstant, rotationSpeedRadiansPerSecond * speedConstant);
        backRightModule.setSpeeds(longitudinalSpeedMetersPerSecond * speedConstant, lateralSpeedMetersPerSecond * speedConstant, rotationSpeedRadiansPerSecond * speedConstant);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxWheelDriveSpeedMetersPerSecond); //TODO Replace with max speed
        frontLeftModule.setState(desiredStates[0]);
        frontRightModule.setState(desiredStates[1]);
        backLeftModule.setState(desiredStates[2]);
        backRightModule.setState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    public void fieldCentricSwerve(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        double offsetRadians = -getGyroAngle().getRadians();
        setModuleSpeeds(
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
        setModuleSpeeds(longitudinalSpeed, 0, 0);
    }

    public void driveLaterally(double lateralSpeed) {
        setModuleSpeeds(0, lateralSpeed, 0);
    }

    public void spin(double speedRadiansPerSecond) {
        setModuleSpeeds(0, 0, speedRadiansPerSecond);
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