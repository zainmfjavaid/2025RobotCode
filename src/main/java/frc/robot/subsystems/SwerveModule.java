package frc.robot.subsystems;

import frc.robot.hardware.KrakenMotor;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.Module;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private static final double noAngleThreshold = 1E-6;

    private final KrakenMotor driveMotor;
    private final KrakenMotor angleMotor;

    public final double turnAngleRadians;

    private final AbsoluteEncoder wheelAngleAbsoluteEncoder;

    private double lastDesiredWheelAngleRadians = 0;

    private final String name;

    public SwerveModule(Module module, InvertedValue driveInvertedValue, InvertedValue angleInvertedValue) {
        driveMotor = new KrakenMotor(module.getDriveMotorDeviceId(), driveInvertedValue);
        angleMotor = new KrakenMotor(module.getAngleMotorDeviceId(), angleInvertedValue);

        double unnormalizedTurnAngleRadians = SwerveUtils.getAngleRadiansFromComponents(module.getLocation().getX(), module.getLocation().getY()) + Math.PI / 2;
        turnAngleRadians = SwerveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(module.getEncoderConfig(), SensorDirectionValue.CounterClockwise_Positive);

        name = module.getName();

        resetEncoders();
    }

    @SuppressWarnings("unused")
    private double getAngleAbsoluteEncoderPositionRadians() {
        return wheelAngleAbsoluteEncoder.getPositionRadians();
    }

    private double getAngleRelativeEncoderPositionRadians() {
        return SwerveUtils.normalizeAngleRadiansSigned(SwerveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
    }

    public void setState(double wheelDriveSpeedMetersPerSecond, double desiredWheelAngleRadians) {
        lastDesiredWheelAngleRadians = desiredWheelAngleRadians;
        
        double currentWheelAngleRadians = 
        // getAngleAbsoluteEncoderPositionRadians();
        getAngleRelativeEncoderPositionRadians();

        double wheelAngleErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;
        // If greater than 180 deg, go the other way
        if (wheelAngleErrorRadians > Math.PI) {
            wheelAngleErrorRadians = Constants.kTau - wheelAngleErrorRadians;
        }
        // If greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelAngleErrorRadians) > Math.PI / 2) {
            wheelAngleErrorRadians = SwerveUtils.normalizeAngleRadiansSigned(wheelAngleErrorRadians + Math.PI);
            wheelDriveSpeedMetersPerSecond = -wheelDriveSpeedMetersPerSecond;
        }

        setAngleMotorRelativeSpeed(SwerveUtils.toAngleRelativeSpeed(wheelAngleErrorRadians));
        
        setDriveMotorRelativeSpeed(SwerveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond));
    }

    public void setState(SwerveModuleState desiredState) {
        setState(desiredState.speedMetersPerSecond, desiredState.angle.getRadians());
    }

    public void setDriveMotorRelativeSpeed(double relativeSpeed) {
        driveMotor.setRelativeSpeed(relativeSpeed);
    }

    public void setAngleMotorRelativeSpeed(double relativeSpeed) {
        angleMotor.setRelativeSpeed(relativeSpeed);
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(SwerveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    // ACCESSORS

    public SwerveModuleState calculateDesiredState(double rXSpeedMetersPerSecond, double rYSpeedMetersPerSecond, double rRotationSpeedRadiansPerSecond) {
        double rotationSpeedMetersPerSecond = rRotationSpeedRadiansPerSecond / SwerveConstants.kMaxRotationSpeedRadiansPerSecond * SwerveConstants.kMaxWheelDriveSpeedMetersPerSecond;
        
        double xSpeedMetersPerSecond = rXSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.cos(turnAngleRadians);
        double ySpeedMetersPerSecond = rYSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.sin(turnAngleRadians);
        
        double driveSpeedMetersPerSecond = Math.hypot(xSpeedMetersPerSecond, ySpeedMetersPerSecond);

        double desiredWheelAngleRadians = lastDesiredWheelAngleRadians;
        if (SwerveUtils.toDriveRelativeSpeed(driveSpeedMetersPerSecond) > noAngleThreshold) {
            desiredWheelAngleRadians = SwerveUtils.normalizeAngleRadiansSigned(SwerveUtils.getAngleRadiansFromComponents(xSpeedMetersPerSecond, ySpeedMetersPerSecond));
        } 

        return new SwerveModuleState(driveSpeedMetersPerSecond, Rotation2d.fromRadians(desiredWheelAngleRadians));
    }

    public SwerveModuleState getState() {
        double speedRadiansPerSecond = SwerveUtils.driveMotorToWheel(driveMotor.getSpeedRadiansPerSecond());
        double speedMetersPerSecond = SwerveUtils.getWheelLinearVelocity(speedRadiansPerSecond);
        Rotation2d angle = Rotation2d.fromRadians(wheelAngleAbsoluteEncoder.getPositionRadians());
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = SwerveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters;
        Rotation2d angle = Rotation2d.fromRadians(SwerveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        return new SwerveModulePosition(distanceMeters, angle);
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(wheelAngleAbsoluteEncoder.getPositionRotations());
    }

    // PRINT ENCODER VALUES

    // rotation of wheel
    public void printEncoderPositions() {
        System.out.print(name + ": ");
        double rRot = SwerveUtils.angleMotorToWheel(angleMotor.getPositionRotations());
        double rRad = SwerveUtils.rotationsToRadians(rRot);
        double aRot = wheelAngleAbsoluteEncoder.getPositionRotations();
        double aRad = SwerveUtils.rotationsToRadians(aRot);
        System.out.println("RRot " + rRot + ", RRad " + rRad + ", ARot " + aRot + ", ARad " + aRad);    
    }
    
    public void printDriveEncoderValue() {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }
}