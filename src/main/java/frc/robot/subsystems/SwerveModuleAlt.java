package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.SparkMaxMotor;

// version of swerve module without helper library imports

public class SwerveModuleAlt {
    private final SparkMaxMotor driveMotor;
    private final SparkMaxMotor angleMotor;

    private final double turnAngleRadians;

    public SwerveModuleAlt(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        driveMotor = new SparkMaxMotor(driveMotorDeviceId, false, false);
        angleMotor = new SparkMaxMotor(angleMotorDeviceId, false, false);
        turnAngleRadians = getTurningAngleRadians(location);
    }

    private static double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - DriveUtils.getAngleRadiansFromComponents(location.getY(), location.getX());
        return DriveUtils.normalizeAngleRadiansSigned(turningAngleRadians);
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(DriveUtils.normalizeSpeed(speed));
    }
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(DriveUtils.normalizeSpeed(speed));
    }

    public void setState(double driveSpeed, double driveAngleRadians, double turnSpeed) {
        double[] desiredState = getDesiredState(driveAngleRadians, turnAngleRadians, driveSpeed, turnSpeed);
        double desiredAngleRadians = desiredState[0];
        double desiredSpeed = desiredState[1];
        setAngle(Rotation2d.fromRadians(desiredAngleRadians));
        setDriveMotorSpeed(desiredSpeed);
    }
    
    // vector addition
    private static double[] getDesiredState(double driveAngleRadians, double turnAngleRadians, double driveSpeed, double turnSpeed) {
        // Get x and y components of speeds
        double driveSpeedY = driveSpeed * Math.sin(driveAngleRadians);
        double driveSpeedX = driveSpeed * Math.cos(driveAngleRadians);
        double turnSpeedY = turnSpeed * Math.sin(turnAngleRadians);
        double turnSpeedX = turnSpeed * Math.cos(turnAngleRadians);
        // Get total speeds in x and y directions
        double speedY = driveSpeedY + turnSpeedY;
        double speedX = driveSpeedX + turnSpeedX;
        // Determine and return angle and total speed
        double desiredAngle = Math.atan2(speedY, speedX);
        double speed = DriveUtils.normalizeSpeed(Math.hypot(speedX, speedY));
        return new double[] {desiredAngle, speed};
    }

    /**
     * Turn the motor to the desired wheel angle
     * @param desiredAngle the desired wheel angle
     */
    public void setAngle(Rotation2d desiredAngle) {
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(desiredAngle.getRadians());
        double wheelErrorRadians = DriveUtils.optimizeErrorRadians(DriveUtils.normalizeAngleRadiansSigned(desiredWheelAngleRadians - currentWheelAngleRadians));
        double speed = DriveUtils.getAngleMotorSpeed(wheelErrorRadians, currentWheelAngleRadians);
        setAngleMotorSpeed(speed);
    }
}
