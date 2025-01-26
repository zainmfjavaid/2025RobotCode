package frc.robot.subsystems;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

public class SwerveModuleSparkMax {
    private final SparkMaxMotor driveMotor;
    private final SparkMaxMotor angleMotor;

    private final AbsoluteEncoder wheelAngleAbsoluteEncoder; // rotations of the wheel, not the motor

    public SwerveModuleSparkMax(int driveMotorDeviceId, int angleMotorDeviceId, EncoderConfig config) {
        driveMotor = new SparkMaxMotor(driveMotorDeviceId, false, false);
        
        // option 1: true, true
        // option 2: false, false
        angleMotor = new SparkMaxMotor(angleMotorDeviceId, true, true);

        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        resetEncoders();
    }

    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(DriveUtils.normalizeSpeed(speed));
    }
    // positive speed is counterclockwise
    public void setAngleMotorSpeed(double speed) {
        angleMotor.set(DriveUtils.normalizeSpeed(speed));
    }

    /**
     * This sets the SwerveModule to the desired state
     * @param state the desired speed and angle
     */
    public void setState(SwerveModuleState state) {
        double driveMotorSpeed = state.speedMetersPerSecond / DriveConstants.kMaxDriveSpeedMetersPerSecond;
        
        double currentWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        double desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(state.angle.getRadians());
        double wheelErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        
        // if greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelErrorRadians) > Math.PI / 2) {
            wheelErrorRadians = DriveUtils.normalizeAngleRadiansSigned(wheelErrorRadians + Math.PI);
            driveMotorSpeed = -driveMotorSpeed;
        }

        setAngleMotorSpeed(DriveUtils.getAngleMotorSpeed(wheelErrorRadians, currentWheelAngleRadians));

        setDriveMotorSpeed(driveMotorSpeed);
    }

    // Set the relative encoder values to default
    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double r2 = angleMotor.getPositionRotations();
        double r1 = DriveUtils.angleMotorToWheel(r2);
        double a1 = wheelAngleAbsoluteEncoder.getPositionRotations();
        double a2 = DriveUtils.angleWheelToMotor(a1);
        System.out.println("R1 " + r1 + ", R2 " + r2 + ", A1 " + a1 + ", A2 " + a2);    
    }

    public void printDriveEncoderValue(String name) {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }

    // meters
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters,
            Rotation2d.fromRadians(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()))
        );
    }
}

