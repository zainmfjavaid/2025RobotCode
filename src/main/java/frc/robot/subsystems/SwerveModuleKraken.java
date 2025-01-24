package frc.robot.subsystems;

import frc.robot.hardware.KrakenMotor;

public class SwerveModuleKraken {
    public final KrakenMotor driveMotor;
    public final KrakenMotor angleMotor;

    public SwerveModuleKraken(int driveMotorDeviceId, int angleMotorDeviceId) {
        driveMotor = new KrakenMotor(driveMotorDeviceId, false, false);
        angleMotor = new KrakenMotor(angleMotorDeviceId, false, false);
    }

    public void setState(double driveSpeed, double angleRadians, double turnSpeed) {
        
    }
}