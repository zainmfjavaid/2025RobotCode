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

// USES ABSOLUTE ENCODER

// future update: make it so that it stays at last angle, when controller goes to 0

public class SwerveModule {
    public final KrakenMotor driveMotor;
    public final KrakenMotor angleMotor;

    public final double turnAngleRadians;

    public final AbsoluteEncoder wheelAngleAbsoluteEncoder;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new KrakenMotor(driveMotorDeviceId, false, false);

        // reverse motor if needed to match direction of absolute encoder
        // (reversing encoder doesn't matter because relative encoder is not used, but it would if it were)
        angleMotor = new KrakenMotor(angleMotorDeviceId, true, true);

        // supplement
        double unnormalizedTurnAngleRadians = DriveUtils.getAngleRadiansFromComponents(location.getX(), location.getY()) + Math.PI / 2;
        turnAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        resetEncoders();
    }

    public void setState(double robotLongitudinalSpeedMetersPerSecond, double robotLateralSpeedMetersPerSecond, double robotRotationSpeedRadiansPerSecond) {        
        double rotationSpeedMetersPerSecond = robotRotationSpeedRadiansPerSecond / DriveConstants.kMaxRotationSpeedRadiansPerSecond * DriveConstants.kMaxWheelDriveSpeedMetersPerSecond;
        
        double longitudinalSpeedMetersPerSecond = robotLongitudinalSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.cos(turnAngleRadians);
        double lateralSpeedMetersPerSecond = robotLateralSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.sin(turnAngleRadians);
        
        double wheelDriveSpeedMetersPerSecond = Math.hypot(lateralSpeedMetersPerSecond, longitudinalSpeedMetersPerSecond);

        double desiredWheelAngleRadians = 0;
        if (DriveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond) > 0.01) {
            desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.getAngleRadiansFromComponents(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond));
        } 

        // no need to normalize because it automatically normalizes
        double currentWheelAngleRadians = wheelAngleAbsoluteEncoder.getPositionRadians();

        double wheelAngleErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;

        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelAngleErrorRadians) > Math.PI / 2) {
            wheelAngleErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelAngleErrorRadians + Math.PI);
            wheelDriveSpeedMetersPerSecond = -wheelDriveSpeedMetersPerSecond;
        }

        desiredWheelAngleRadians = currentWheelAngleRadians + wheelAngleErrorRadians;
        
        double wheelAngleSpeedRadiansPerSecond = TeleopSwerveConstants.kRotationController.calculate(currentWheelAngleRadians, desiredWheelAngleRadians);        
        double angleMotorRelativeSpeed = DriveUtils.radiansToRotations(wheelAngleSpeedRadiansPerSecond);
        setAngleMotorRelativeSpeed(angleMotorRelativeSpeed);
        
        double driveMotorRelativeSpeed = DriveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond);
        setDriveMotorRelativeSpeed(driveMotorRelativeSpeed);
    }

    public void setDriveMotorRelativeSpeed(double relativeSpeed) {
        driveMotor.setRelativeSpeed(relativeSpeed);
    }

    public void setAngleMotorRelativeSpeed(double relativeSpeed) {
        angleMotor.setRelativeSpeed(relativeSpeed);
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        // angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * DriveConstants.kWheelRadiusMeters;
        Rotation2d angle = Rotation2d.fromRadians(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        return new SwerveModulePosition(distanceMeters, angle);
    }

    // print encoder
    

    // rotation of wheel
    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double rRot = DriveUtils.angleMotorToWheel(angleMotor.getPositionRotations());
        double rRad = DriveUtils.rotationsToRadians(rRot);
        double aRot = wheelAngleAbsoluteEncoder.getPositionRotations();
        double aRad = DriveUtils.rotationsToRadians(aRot);
        System.out.println("RRot " + rRot + ", RRad " + rRad + ", ARot " + aRot + ", ARad " + aRad);    
    }

    public void printDriveEncoderValue(String name) {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }
}