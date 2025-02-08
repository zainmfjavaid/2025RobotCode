package frc.robot.subsystems;

import frc.robot.hardware.KrakenMotor;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
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

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new KrakenMotor(driveMotorDeviceId, false, false);
        
        // option 1: true, true // option 2: false, false
        angleMotor = new KrakenMotor(angleMotorDeviceId, false, false);

        double unnormalizedTurnAngleRadians = (Math.PI / 2) - DriveUtils.getAngleRadiansFromComponents(location.getY(), location.getX());
        turnAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        resetEncoders();
    }

    public void setState(double robotLongitundalSpeed, double robotLateralSpeed, double robotRotationSpeed) {
        // speeds
        double longitundalSpeed = robotLongitundalSpeed + robotRotationSpeed * Math.sin(turnAngleRadians);
        double lateralSpeed = robotLateralSpeed + robotRotationSpeed * Math.cos(turnAngleRadians);
        
        double driveMotorSpeed = Math.hypot(lateralSpeed, longitundalSpeed);
        
        // angles
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.getAngleRadiansFromComponents(longitundalSpeed, lateralSpeed));
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        
        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelErrorRadians + Math.PI);
            driveMotorSpeed = -driveMotorSpeed;
        }
        
        double wheelAngleSpeed = TeleopSwerveConstants.kRotationController.calculate(currentWheelAngleRadians, desiredWheelAngleRadians);
        setAngleMotorSpeed(DriveUtils.angleWheelToMotor(wheelAngleSpeed));

        setDriveMotorSpeed(driveMotorSpeed);
    }

    public void setDriveMotorSpeed(double speedMetersPerSecond) {
        driveMotor.set(DriveUtils.normalizeSpeed(speedMetersPerSecond / DriveConstants.kMaxDriveSpeedMetersPerSecond));
    }

    public void setAngleMotorSpeed(double speedRadiansPerSecond) {
        angleMotor.set(DriveUtils.normalizeSpeed(speedRadiansPerSecond / SwerveConstants.kMaxRotationSpeedRadiansPerSecond));
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters;
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