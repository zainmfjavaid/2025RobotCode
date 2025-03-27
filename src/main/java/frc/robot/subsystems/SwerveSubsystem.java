package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveConstants.TeleopSwerveConstants;
import frc.robot.Constants.SwerveConstants.Module;
import frc.robot.hardware.Controller.DriverController;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveSubsystem extends SubsystemBase {
    // Modules
    
    private final SwerveModule frontLeftModule = new SwerveModule(Module.FRONT_LEFT, true, true);
    private final SwerveModule frontRightModule = new SwerveModule(Module.FRONT_RIGHT, false, true);
    private final SwerveModule backLeftModule = new SwerveModule(Module.BACK_LEFT, true, true);
    private final SwerveModule backRightModule = new SwerveModule(Module.BACK_RIGHT, false, true); 

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Module.FRONT_LEFT.getLocation(), Module.FRONT_RIGHT.getLocation(), Module.BACK_LEFT.getLocation(), Module.BACK_RIGHT.getLocation());

    private final Pigeon2 gyro = new Pigeon2(20, "CANivore2158");
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(getKinematics(), new Rotation2d(0), getModulePositions());

    private final double kRetractElevatorThresholdRadians = Rotation2d.fromDegrees(3).getRadians();

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
            for (int i = 0; i < 4; i++) {
                moduleStates[i] = new SwerveModuleState(0, Rotation2d.fromRadians(offsetRadians));
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
        if (rollRadians > kRetractElevatorThresholdRadians || pitchRadians > kRetractElevatorThresholdRadians) {
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

    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
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

    public void setDriveVoltage(double volts) {
        frontLeftModule.setDriveVoltage(volts);
        frontRightModule.setDriveVoltage(volts);
        backLeftModule.setDriveVoltage(volts);
        backRightModule.setDriveVoltage(volts);
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

    public void printStates() {
        frontLeftModule.printState();
        frontRightModule.printState();
        backLeftModule.printState();
        backRightModule.printState();
    }

    public void printVoltages() {
        frontLeftModule.printDriveVoltage();
        frontRightModule.printDriveVoltage();
        backLeftModule.printDriveVoltage();
        backRightModule.printDriveVoltage();
    }
}