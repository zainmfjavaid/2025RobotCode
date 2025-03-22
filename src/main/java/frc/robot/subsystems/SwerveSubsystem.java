package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.TeleopSwerveConstants;
import frc.robot.Constants.SwerveConstants.Module;
import frc.robot.hardware.Controller.DriverController;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class SwerveSubsystem extends SubsystemBase {
    // Modules
    private final SwerveModule frontLeftModule = new SwerveModule(Module.FRONT_LEFT, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive);
    private final SwerveModule frontRightModule = new SwerveModule(Module.FRONT_RIGHT, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive);
    private final SwerveModule backLeftModule = new SwerveModule(Module.BACK_LEFT, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive);
    private final SwerveModule backRightModule = new SwerveModule(Module.BACK_RIGHT, InvertedValue.CounterClockwise_Positive, InvertedValue.CounterClockwise_Positive); 

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Module.FRONT_LEFT.getLocation(), Module.FRONT_RIGHT.getLocation(), Module.BACK_LEFT.getLocation(), Module.BACK_RIGHT.getLocation());

    private final Pigeon2 gyro = new Pigeon2(20, "CANivore2158");
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(getKinematics(), new Rotation2d(0), getModulePositions());

    private static RobotConfig config;

    private final double retractElevatorThresholdRadians = Rotation2d.fromDegrees(3).getRadians();

    private double speedConstant = 1.0;

    public SwerveSubsystem() {
    }

    public void toggleSpeedConstant() {
        if (speedConstant == 1.0) {
            speedConstant = 0.25;
        } else {
            speedConstant = 1.0;
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredModuleStates) {
        SwerveUtils.normalizeModuleStates(desiredModuleStates);
        frontLeftModule.setState(desiredModuleStates[0]);
        frontRightModule.setState(desiredModuleStates[1]);
        backLeftModule.setState(desiredModuleStates[2]);
        backRightModule.setState(desiredModuleStates[3]);
    }

    public SwerveModuleState[] calculateModuleStates(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        return new SwerveModuleState[] {
            frontLeftModule.calculateDesiredState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond),
            frontRightModule.calculateDesiredState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond),
            backLeftModule.calculateDesiredState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond),
            backRightModule.calculateDesiredState(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond)
        };
    }

    public void robotCentricSwerve(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        longitudinalSpeedMetersPerSecond *= speedConstant;
        lateralSpeedMetersPerSecond *= speedConstant;
        rotationSpeedRadiansPerSecond *= speedConstant;
        
        setModuleStates(calculateModuleStates(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond, rotationSpeedRadiansPerSecond));
    }

    public void fieldCentricSwerve(double longitudinalSpeedMetersPerSecond, double lateralSpeedMetersPerSecond, double rotationSpeedRadiansPerSecond) {
        longitudinalSpeedMetersPerSecond *= speedConstant;
        lateralSpeedMetersPerSecond *= speedConstant;
        rotationSpeedRadiansPerSecond *= speedConstant;
        
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        
        double offsetRadians = -getGyroAngle().getRadians();

        if (longitudinalSpeedMetersPerSecond == 0 && lateralSpeedMetersPerSecond == 0 && rotationSpeedRadiansPerSecond == 0) {
            // Set module angles based on gyro
            for (SwerveModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond = 0;
                moduleState.angle = Rotation2d.fromRadians(offsetRadians);
            }
        } else {
            moduleStates = calculateModuleStates(
                // Rotation matrix
                longitudinalSpeedMetersPerSecond * Math.cos(offsetRadians) - lateralSpeedMetersPerSecond * Math.sin(offsetRadians),
                longitudinalSpeedMetersPerSecond * Math.sin(offsetRadians) + lateralSpeedMetersPerSecond * Math.cos(offsetRadians),
                rotationSpeedRadiansPerSecond
            );
        }

        setModuleStates(moduleStates);
    }

    public void swerveDriveTeleop(DriverController driverController) {
        double leftStickY = driverController.getLeftStickY();
        double leftStickX = driverController.getLeftStickX();
        double rightStickX = driverController.getRightStickX();

        leftStickY = Math.abs(leftStickY) > 0.01 ? leftStickY : 0;
        leftStickX = Math.abs(leftStickX) > 0.01 ? leftStickX : 0;
        rightStickX = Math.abs(rightStickX) > 0.01 ? rightStickX : 0;

        fieldCentricSwerve(
            leftStickY * TeleopSwerveConstants.kMaxDriveSpeedMetersPerSecond,
            leftStickX * TeleopSwerveConstants.kMaxDriveSpeedMetersPerSecond,
            rightStickX * TeleopSwerveConstants.kMaxRotationSpeedRadiansPerSecond
        );
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    // Accessors

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
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

    // counterclockwise is positive // yaw
    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation3d getGyroRotation3d() {
        return gyro.getRotation3d();
    }

    public void tipDetection() {
        double rollRadians = getGyroRotation3d().getX();
        double pitchRadians = getGyroRotation3d().getY();
        if (rollRadians > retractElevatorThresholdRadians || pitchRadians > retractElevatorThresholdRadians) {
            // System.out.println("tip");
            // elevator.setPosition(IntakeState.STOW);
        }
    }
    
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // gyro and odometer

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

    // set speed

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
        robotCentricSwerve(longitudinalSpeed, 0, 0);
    }

    public void driveLaterally(double lateralSpeed) {
        robotCentricSwerve(0, lateralSpeed, 0);
    }

    public void spin(double speedRadiansPerSecond) {
        robotCentricSwerve(0, 0, speedRadiansPerSecond);
    }

    public void stop() {
        robotCentricSwerve(0, 0, 0);
    }

    // print

    public void printEncoderValues() {
        System.out.println("Angle Encoder Positions");
        frontLeftModule.printEncoderPositions();
        frontRightModule.printEncoderPositions();
        backLeftModule.printEncoderPositions();
        backRightModule.printEncoderPositions();
    }

    public void printDriveEncoderValues() {
        System.out.println("Drive Encoder Positions");
        frontLeftModule.printDriveEncoderValue();
        frontRightModule.printDriveEncoderValue();
        backLeftModule.printDriveEncoderValue();
        backRightModule.printDriveEncoderValue();
    }

    public void printGyroValue() {
        System.out.println("Gyro Value: " + getGyroAngle());
    }

    public void printOdometerPose() {
        System.out.println("Odometer Pose: " + getPose());
    }
    
    // auton

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