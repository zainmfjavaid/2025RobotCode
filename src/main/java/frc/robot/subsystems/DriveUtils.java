package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleopSwerveConstants;

public class DriveUtils {
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

    /**
     * Optimize the error so that the motor moves the shorter direction
     * If the error is greater than pi, the other direction is shorter, so flip the angle
     * @param errorRadians the difference from the desired angle and the current angle
     * @return the optimized error
     */
    public static double optimizeErrorRadians(double errorRadians) {
        return errorRadians > Math.PI ? errorRadians = -(Constants.kTau - errorRadians) : errorRadians;
    }

     /**
     * Convert the angle to be between -pi and pi
     * @param angleRadians the unnormalized angle radians
     * @return the normalized angle radians
     */
    public static double normalizeAngleRadiansSigned(double angleRadians) {
        while (angleRadians < -Math.PI || angleRadians > Math.PI) {
            angleRadians += angleRadians < -Math.PI ? Constants.kTau : -Constants.kTau;
        }
        return angleRadians;
    }
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
}
