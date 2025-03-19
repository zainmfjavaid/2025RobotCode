package frc.robot.subsystems;

import frc.robot.hardware.KrakenMotor;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopSwerveConstants;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveModule {
    public final KrakenMotor driveMotor;
    public final KrakenMotor angleMotor;

    public final double turnAngleRadians;

    public final AbsoluteEncoder wheelAngleAbsoluteEncoder;

    public double lastAngleRadians = 0;

    private final String name;

    public SwerveModule(Module module, boolean reverseDrive, boolean reverseAngle, String name) {
        driveMotor = new KrakenMotor(module.getDriveMotorDeviceId(), reverseDrive, reverseDrive);

        // reverse motor if needed to match direction of absolute encoder
        // (reversing encoder doesn't matter because relative encoder is not used, but it would if it were)
        angleMotor = new KrakenMotor(module.getAngleMotorDeviceId(), reverseAngle, reverseAngle);

        double unnormalizedTurnAngleRadians = (Math.PI / 2) - DriveUtils.getAngleRadiansFromComponents(location.getY(), location.getX());
        turnAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(module.getEncoderConfig(), SensorDirectionValue.CounterClockwise_Positive);

        this.name = name;

        resetEncoders();
    }

    public void setState(double robotLongitudinalSpeedMetersPerSecond, double robotLateralSpeedMetersPerSecond, double robotRotationSpeedRadiansPerSecond) {
        double rotationSpeedMetersPerSecond = robotRotationSpeedRadiansPerSecond / DriveConstants.kMaxRotationSpeedRadiansPerSecond * DriveConstants.kMaxWheelDriveSpeedMetersPerSecond;
        
        double longitudinalSpeedMetersPerSecond = robotLongitudinalSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.sin(turnAngleRadians);
        double lateralSpeedMetersPerSecond = robotLateralSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.cos(turnAngleRadians);

        double wheelDriveSpeedMetersPerSecond = Math.hypot(lateralSpeedMetersPerSecond, longitudinalSpeedMetersPerSecond);

        double desiredWheelAngleRadians = lastAngleRadians;
        if (DriveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond) > 1E-6) {
            desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.getAngleRadiansFromComponents(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond));
        } 

        setState(wheelDriveSpeedMetersPerSecond, desiredWheelAngleRadians);
    }

    public void stop() {
        setAngleMotorRelativeSpeed(0);
        setDriveMotorRelativeSpeed(0);
    }

    public void setState(double wheelDriveSpeedMetersPerSecond, double desiredWheelAngleRadians) {        
        double currentWheelAngleRadians = 
        getRelativeAnglePositionRadians();
        // getAbsoluteAnglePositionRadians();
        
        double wheelAngleErrorRadians = (desiredWheelAngleRadians) - (currentWheelAngleRadians);

        System.out.println(desiredWheelAngleRadians + " " + getRelativeAnglePositionRadians() + " " + getAbsoluteAnglePositionRadians());

        // If greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelAngleErrorRadians) > Math.PI / 2) {
            wheelAngleErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelAngleErrorRadians + Math.PI);
            wheelDriveSpeedMetersPerSecond = -wheelDriveSpeedMetersPerSecond;
        }
        
        double wheelAngleSpeedRadiansPerSecond = TeleopSwerveConstants.kRotationController.calculate(currentWheelAngleRadians, desiredWheelAngleRadians);
        double angleMotorRelativeSpeed = wheelAngleSpeedRadiansPerSecond / DriveConstants.kMaxWheelAngleSpeedRadiansPerSecond;
        setAngleMotorRelativeSpeed(angleMotorRelativeSpeed);
        
        setDriveMotorRelativeSpeed(wheelDriveSpeedMetersPerSecond / DriveConstants.kMaxWheelDriveSpeedMetersPerSecond);
    }

    public void setDriveMotorRelativeSpeed(double relativeSpeed) {
        driveMotor.set(relativeSpeed);
    }

    public void setAngleMotorRelativeSpeed(double relativeSpeed) {
        angleMotor.set(relativeSpeed);
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
        System.out.println(name + " " + wheelAngleAbsoluteEncoder.getPositionRadians());
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * DriveConstants.kWheelRadiusMeters;
        Rotation2d angle = Rotation2d.fromRadians(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        return new SwerveModulePosition(distanceMeters, angle);
    }

    // print encoder

    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double rRot = angleMotor.getPositionRotations();
        double rRad = DriveUtils.angleMotorToWheel(rRot);
        double aRot = wheelAngleAbsoluteEncoder.getPositionRotations();
        double aRad = DriveUtils.angleWheelToMotor(aRot);
        System.out.println("RRot " + rRot + ", RRad " + rRad + ", ARot " + aRot + ", ARad " + aRad);    
    }

    public void printDriveEncoderValue(String name) {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }
}