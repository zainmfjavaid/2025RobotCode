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

public class SwerveModuleKraken {
    public final KrakenMotor driveMotor;
    public final KrakenMotor angleMotor;

    public final double turnAngleRadians;

    public final AbsoluteEncoder wheelAngleAbsoluteEncoder;

    public SwerveModuleKraken(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new KrakenMotor(driveMotorDeviceId, false, false);
        angleMotor = new KrakenMotor(angleMotorDeviceId, false, false);

        double unnormalizedTurnAngleRadians = (Math.PI / 2) - DriveUtils.getAngleRadiansFromComponents(location.getY(), location.getX());
        turnAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        resetEncoders();
    }

    public void setState(double robotLongitundalSpeed, double robotLateralSpeed, double robotRotationSpeed) {
        double longitundalSpeed = robotLongitundalSpeed + robotRotationSpeed * Math.sin(turnAngleRadians);
        double lateralSpeed = robotLateralSpeed + robotRotationSpeed * Math.cos(turnAngleRadians);
        
        double driveMotorSpeed = Math.hypot(lateralSpeed, longitundalSpeed);
        
        double unnormalizedDesiredWheelAngleRadians = DriveUtils.getAngleRadiansFromComponents(longitundalSpeed, lateralSpeed);

        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedDesiredWheelAngleRadians);
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        
        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelErrorRadians + Math.PI);
            driveMotorSpeed = -driveMotorSpeed;
        }
        
        double wheelAngleSpeed = TeleopSwerveConstants.kRotationController.calculate(currentWheelAngleRadians, desiredWheelAngleRadians);
        angleMotor.set(DriveUtils.angleWheelToMotor(wheelAngleSpeed));

        driveMotor.set(driveMotorSpeed / DriveConstants.kMaxDriveSpeedMetersPerSecond);
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
}