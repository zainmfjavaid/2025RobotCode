package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.TeleopSwerveConstants;

public class SwerveUtils {
    public static double angleWheelToMotor(double value) {
        return value / SwerveConstants.kAngleMotorGearRatio;
    }
    public static double angleMotorToWheel(double value) {
        return value * SwerveConstants.kAngleMotorGearRatio;
    }

    public static double driveWheelToMotor(double value) {
        return value / SwerveConstants.kDriveMotorGearRatio;
    }
    public static double driveMotorToWheel(double value) {
        return value * SwerveConstants.kDriveMotorGearRatio;
    }

    public static double rotationsToRadians(double rotations) {
        return rotations * Constants.kTau;
    }
    public static double radiansToRotations(double radians) {
        return radians / Constants.kTau;
    }

    /**
     * Limit the speed to the range -maxSpeed to maxSpeed
     * @param speed the unnormalized speed
     * @return the normalized speed
     */
    public static double normalizeSpeed(double speed) {
        if (speed > 1) {
            return 1;
        } else if (speed < -1) {
            return -1;
        }
        return speed;
    }

    // Convert to between -pi and pi
    public static double normalizeAngleRadiansSigned(double angleRadians) {
        while (angleRadians < -Math.PI || angleRadians > Math.PI) {
            angleRadians += angleRadians < -Math.PI ? Constants.kTau : -Constants.kTau;
        }
        return angleRadians;
    }
    // Convert to between 0 and 2pi
    public static double normalizeAngleRadiansUnsigned(double angleRadians) {
        while (angleRadians < 0 || angleRadians > Constants.kTau) {
            angleRadians += angleRadians < 0 ? Constants.kTau : -Constants.kTau;
        }
        return angleRadians;
    }

    public static double getAngleMotorSpeed(double wheelErrorRadians, double currentWheelAngleRadians) {
        double motorErrorRadians = angleWheelToMotor(wheelErrorRadians);
        double currentMotorAngleRadians = angleWheelToMotor(currentWheelAngleRadians);
        double desiredMotorAngleRadians = currentMotorAngleRadians + motorErrorRadians;
        return TeleopSwerveConstants.kRotationController.calculate(motorErrorRadians, desiredMotorAngleRadians);
    }

    // Return the angle in radians formed by the x and y components
    // x is longitudinal, y is lateral
    // y is opposite, x is adjacent
    public static double getAngleRadiansFromComponents(double x, double y) {
        return normalizeAngleRadiansSigned(Math.atan2(y, x));
    }

    public static double toDriveRelativeSpeed(double driveSpeedMetersPerSecond) {
        return driveSpeedMetersPerSecond / SwerveConstants.kMaxWheelDriveSpeedMetersPerSecond;
    }
    public static double toAngleRelativeSpeed(double angleSpeedRadiansPerSecond) {
        return angleSpeedRadiansPerSecond / SwerveConstants.kMaxWheelAngleSpeedRadiansPerSecond;
    }

    public static double getLinearVelocity(double radiansPerSecond, double radius) {
        return radiansPerSecond * radius;
    } 
    public static double getWheelLinearVelocity(double radiansPerSecond) {
        return getLinearVelocity(radiansPerSecond, SwerveConstants.kWheelRadiusMeters);
    }
}
