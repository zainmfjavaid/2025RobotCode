package frc.robot.subsystems;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

public class SwerveModule {
    private final Motor driveMotor;
    private final Motor angleMotor;

    private final AbsoluteEncoder wheelAngleAbsoluteEncoder; // rotations of the wheel, not the motor

    private final double turnAngleRadians;

    public SwerveModule(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig config) {
        // does the type of battery or position affect this?
        driveMotor = new Motor(driveMotorDeviceId, false, false);
        // option 1: true, true
        // option 2: false, false
        angleMotor = new Motor(angleMotorDeviceId, true, true);

        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(config, SensorDirectionValue.CounterClockwise_Positive);

        turnAngleRadians = getTurningAngleRadians(location); // only used for alternative swerve

        resetEncoders();
    }

    private static double getTurningAngleRadians(Translation2d location) {
        double turningAngleRadians = (Math.PI / 2) - getAngleRadiansFromComponents(location.getY(), location.getX());
        return DriveUtils.normalizeAngleRadiansSigned(turningAngleRadians);
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

    public void setState(double driveSpeed, double driveAngleRadians, double turnSpeed) {
        double[] desiredState = getDesiredState(driveAngleRadians, turnAngleRadians, driveSpeed, turnSpeed);
        double desiredAngleRadians = desiredState[0];
        double desiredSpeed = desiredState[1];
        setAngle(Rotation2d.fromRadians(desiredAngleRadians));
        setDriveMotorSpeed(desiredSpeed);
    }

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

    // Return the angle in radians formed by the x and y components
    public static double getAngleRadiansFromComponents(double y, double x) {
        return DriveUtils.normalizeAngleRadiansSigned(Math.atan2(y, x));
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

