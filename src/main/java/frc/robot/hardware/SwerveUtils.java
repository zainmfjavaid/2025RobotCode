package frc.robot.hardware;

import frc.robot.Constants;

public class SwerveUtils {
    public static double angleWheelToMotor(double value) {
        return value / Constants.kAngleMotorGearRatio;
    }
    public static double angleMotorToWheel(double value) {
        return value * Constants.kAngleMotorGearRatio;
    }

    public static double driveWheelToMotor(double value) {
        return value / Constants.kDriveMotorGearRatio;
    }
    public static double driveMotorToWheel(double value) {
        return value * Constants.kDriveMotorGearRatio;
    }

    public static double rotationsToRadians(double rotations) {
        return rotations * Constants.kTau;
    }
    public static double radiansToRotations(double radians) {
        return radians / Constants.kTau;
    }
}
