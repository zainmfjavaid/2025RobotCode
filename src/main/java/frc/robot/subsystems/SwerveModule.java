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

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new KrakenMotor(driveMotorDeviceId, false, false);

        // change the reverse motor and reverse encoder booleans if needed
        angleMotor = new KrakenMotor(angleMotorDeviceId, true, true);

        double unnormalizedTurnAngleRadians = (Math.PI / 2) - DriveUtils.getAngleRadiansFromComponents(location.getY(), location.getX());
        turnAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);
    }

    public void setState(double robotLongitudinalSpeedMetersPerSecond, double robotLateralSpeedMetersPerSecond, double robotRotationSpeedRadiansPerSecond) {
        double rotationSpeedMetersPerSecond = robotRotationSpeedRadiansPerSecond / DriveConstants.kMaxRotationSpeedRadiansPerSecond * DriveConstants.kMaxWheelDriveSpeedMetersPerSecond;
        
        double longitudinalSpeedMetersPerSecond = robotLongitudinalSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.sin(turnAngleRadians);
        double lateralSpeedMetersPerSecond = robotLateralSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.cos(turnAngleRadians);
        System.out.println("A:" + longitudinalSpeedMetersPerSecond + " " + lateralSpeedMetersPerSecond);

        double wheelDriveSpeedMetersPerSecond = Math.hypot(lateralSpeedMetersPerSecond, longitudinalSpeedMetersPerSecond);
        
        // angles in radians
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.getAngleRadiansFromComponents(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond));
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(wheelAngleAbsoluteEncoder.getPositionRadians());

        double wheelAngleErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        
        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelAngleErrorRadians) > Math.PI / 2) {
            wheelAngleErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelAngleErrorRadians + Math.PI);
            wheelDriveSpeedMetersPerSecond = -wheelDriveSpeedMetersPerSecond;
        }
        
        double wheelAngleSpeedRadiansPerSecond = TeleopSwerveConstants.kRotationController.calculate(currentWheelAngleRadians, desiredWheelAngleRadians);
        
        System.out.println("B: " + wheelAngleSpeedRadiansPerSecond + " " + DriveConstants.kMaxWheelAngleSpeedRadiansPerSecond);
        
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