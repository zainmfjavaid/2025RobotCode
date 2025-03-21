package frc.robot.subsystems;

import frc.robot.hardware.KrakenMotor;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopSwerveConstants;
import frc.robot.Constants.SwerveConstants.Module;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// USES ABSOLUTE ENCODER
// don't use angle motor relative encoder

// current issues: the robot doesn't go completely straight, it turns a bit
// the issue is not with the gyro but with my code or the absolute encoder offsets

public class SwerveModule {
    public final KrakenMotor driveMotor;
    public final KrakenMotor angleMotor;

    public final double turnAngleRadians;

    public final AbsoluteEncoder wheelAngleAbsoluteEncoder;

    public double lastAngleRadians = 0;

    private final String name;

    public SwerveModule(Module module, boolean reverseDrive, boolean reverseAngle, String name) {
        driveMotor = new KrakenMotor(module.getDriveMotorDeviceId(), reverseDrive, reverseDrive);

        // reverse motor if needed to match direction of absolute encoder
        // (reversing encoder doesn't matter because relative encoder is not used, but it would if it were)
        angleMotor = new KrakenMotor(module.getAngleMotorDeviceId(), reverseAngle, reverseAngle);

        double unnormalizedTurnAngleRadians = DriveUtils.getAngleRadiansFromComponents(module.getLocation().getX(), module.getLocation().getY()) + Math.PI / 2;
        turnAngleRadians = DriveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(module.getEncoderConfig(), SensorDirectionValue.CounterClockwise_Positive);

        this.name = name;

        resetEncoders();
    }

    public void setSpeeds(double robotLongitudinalSpeedMetersPerSecond, double robotLateralSpeedMetersPerSecond, double robotRotationSpeedRadiansPerSecond) {
        double rotationSpeedMetersPerSecond = robotRotationSpeedRadiansPerSecond / DriveConstants.kMaxRotationSpeedRadiansPerSecond * DriveConstants.kMaxWheelDriveSpeedMetersPerSecond;
        
        double longitudinalSpeedMetersPerSecond = robotLongitudinalSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.cos(turnAngleRadians);
        double lateralSpeedMetersPerSecond = robotLateralSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.sin(turnAngleRadians);
        
        double wheelDriveSpeedMetersPerSecond = Math.hypot(lateralSpeedMetersPerSecond, longitudinalSpeedMetersPerSecond);

        double desiredWheelAngleRadians = lastAngleRadians;
        if (DriveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond) > 1E-6) {
            desiredWheelAngleRadians = DriveUtils.normalizeAngleRadiansSigned(DriveUtils.getAngleRadiansFromComponents(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond));
        } 

        setState(wheelDriveSpeedMetersPerSecond, desiredWheelAngleRadians);
    }

    public void stop() {
        setAngleMotorRelativeSpeed(0);
        setDriveMotorRelativeSpeed(0);
    }

    public void setState(double wheelDriveSpeedMetersPerSecond, double desiredWheelAngleRadians) {        
        double currentWheelAngleRadians = 
        getRelativeAnglePositionRadians();
        // getAbsoluteAnglePositionRadians();
        
        double wheelAngleErrorRadians = (desiredWheelAngleRadians) - (currentWheelAngleRadians);

        System.out.println(desiredWheelAngleRadians + " " + getRelativeAnglePositionRadians() + " " + getAbsoluteAnglePositionRadians());

        // If greater than 90 deg, add 180 deg and flip drive motor direction
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

        lastAngleRadians = desiredWheelAngleRadians;
    }

    public void setState(SwerveModuleState desiredState) {
        setState(desiredState.speedMetersPerSecond, desiredState.angle.getRadians());
    }

    public SwerveModuleState getState() {
        double speedRadiansPerSecond = DriveUtils.driveMotorToWheel(driveMotor.getSpeedRadiansPerSecond());
        double speedMetersPerSecond = DriveUtils.getWheelLinearVelocity(speedRadiansPerSecond);
        Rotation2d angle = Rotation2d.fromRadians(wheelAngleAbsoluteEncoder.getPositionRadians());
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public double getAbsoluteAnglePositionRadians() {
        return wheelAngleAbsoluteEncoder.getPositionRadians();
    }

    public double getRelativeAnglePositionRadians() {
        return DriveUtils.normalizeAngleRadiansSigned(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
    }

    public void setDriveMotorRelativeSpeed(double relativeSpeed) {
        driveMotor.setRelativeSpeed(relativeSpeed);
    }

    public void setAngleMotorRelativeSpeed(double relativeSpeed) {
        angleMotor.setRelativeSpeed(relativeSpeed);
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(DriveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
        System.out.println(name + " " + wheelAngleAbsoluteEncoder.getPositionRadians());
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = DriveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * DriveConstants.kWheelRadiusMeters;
        Rotation2d angle = Rotation2d.fromRadians(DriveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        return new SwerveModulePosition(distanceMeters, angle);
    }

    // Print encoder values

    // rotation of wheel
    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double rRot = DriveUtils.angleMotorToWheel(angleMotor.getPositionRotations());
        double rRad = getRelativeAnglePositionRadians();
        double aRot = wheelAngleAbsoluteEncoder.getPositionRotations();
        double aRad = DriveUtils.rotationsToRadians(aRot);
        System.out.println("RRot " + rRot + ", RRad " + rRad + ", ARot " + aRot + ", ARad " + aRad);    
    }

    public void printDriveEncoderValue(String name) {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }
}