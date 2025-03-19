package frc.robot.subsystems;

import frc.robot.hardware.KrakenMotor;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.TeleopSwerveConstants;
import frc.robot.Constants.SwerveConstants.Module;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

// USES ABSOLUTE ENCODER
// don't use angle motor relative encoder

// current issues: the robot doesn't go completely straight, it turns a bit
// the issue is not with the gyro but with my code or the absolute encoder offsets

public class SwerveModule {
    private final KrakenMotor driveMotor;
    private final KrakenMotor angleMotor;

    public final double turnAngleRadians;

    private final AbsoluteEncoder wheelAngleAbsoluteEncoder;

    private double lastAngleRadians = 0;

    private final String name;

    private final GenericEntry absoluteAngleEntry;
    private final GenericEntry relativeAngleEntry;

    private final GenericEntry desiredAngleEntry;

    private final GenericEntry driveSpeedEntry;
    private final GenericEntry angleSpeedEntry;

    public SwerveModule(Module module, ShuffleboardLayout absoluteAnglesLayout, ShuffleboardLayout relativeAnglesLayout, ShuffleboardLayout desiredAnglesLayout, ShuffleboardLayout driveSpeedsLayout, ShuffleboardLayout angleSpeedsLayout) {
        driveMotor = new KrakenMotor(module.getDriveMotorDeviceId(), true);
        angleMotor = new KrakenMotor(module.getAngleMotorDeviceId(), true);

        double unnormalizedTurnAngleRadians = SwerveUtils.getAngleRadiansFromComponents(module.getLocation().getX(), module.getLocation().getY()) + Math.PI / 2;
        turnAngleRadians = SwerveUtils.normalizeAngleRadiansSigned(unnormalizedTurnAngleRadians);
        
        wheelAngleAbsoluteEncoder = new AbsoluteEncoder(module.getEncoderConfig(), SensorDirectionValue.CounterClockwise_Positive);

        name = module.getName();

        absoluteAngleEntry = absoluteAnglesLayout.add(name, 0).withWidget(BuiltInWidgets.kGyro).getEntry();
        relativeAngleEntry = relativeAnglesLayout.add(name, 0).withWidget(BuiltInWidgets.kGyro).getEntry();
        
        desiredAngleEntry = desiredAnglesLayout.add(name, 0).withWidget(BuiltInWidgets.kGyro).getEntry();
        
        driveSpeedEntry = driveSpeedsLayout.add(name, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();
        angleSpeedEntry = angleSpeedsLayout.add(name, 0).withWidget(BuiltInWidgets.kNumberBar).getEntry();

        resetEncoders();
    }

    public void setSpeeds(double robotLongitudinalSpeedMetersPerSecond, double robotLateralSpeedMetersPerSecond, double robotRotationSpeedRadiansPerSecond) {
        double rotationSpeedMetersPerSecond = robotRotationSpeedRadiansPerSecond / SwerveConstants.kMaxRotationSpeedRadiansPerSecond * SwerveConstants.kMaxWheelDriveSpeedMetersPerSecond;
        
        double longitudinalSpeedMetersPerSecond = robotLongitudinalSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.cos(turnAngleRadians);
        double lateralSpeedMetersPerSecond = robotLateralSpeedMetersPerSecond + rotationSpeedMetersPerSecond * Math.sin(turnAngleRadians);
        
        double wheelDriveSpeedMetersPerSecond = Math.hypot(lateralSpeedMetersPerSecond, longitudinalSpeedMetersPerSecond);

        double desiredWheelAngleRadians = lastAngleRadians;
        if (SwerveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond) > 1E-6) {
            desiredWheelAngleRadians = SwerveUtils.normalizeAngleRadiansSigned(SwerveUtils.getAngleRadiansFromComponents(longitudinalSpeedMetersPerSecond, lateralSpeedMetersPerSecond));
        } 

        setState(wheelDriveSpeedMetersPerSecond, desiredWheelAngleRadians);
    }

    public void setState(double wheelDriveSpeedMetersPerSecond, double desiredWheelAngleRadians) {  
        double currentWheelAngleRadians = getAngleAbsoluteEncoderPositionRadians();

        absoluteAngleEntry.setDouble(Units.radiansToDegrees(currentWheelAngleRadians));
        relativeAngleEntry.setDouble(Units.radiansToDegrees(getAngleRelativeEncoderPositionRadians()));
        
        desiredAngleEntry.setDouble(Units.radiansToDegrees(desiredWheelAngleRadians));

        double wheelAngleErrorRadians = desiredWheelAngleRadians - currentWheelAngleRadians;

        // If greater than 90 deg, add 180 deg and flip drive motor direction
        if (Math.abs(wheelAngleErrorRadians) > Math.PI / 2) {
            wheelAngleErrorRadians = SwerveUtils.normalizeAngleRadiansSigned(wheelAngleErrorRadians + Math.PI);
            wheelDriveSpeedMetersPerSecond = -wheelDriveSpeedMetersPerSecond;
        }

        desiredWheelAngleRadians = currentWheelAngleRadians + wheelAngleErrorRadians;
        
        double wheelAngleSpeedRadiansPerSecond = TeleopSwerveConstants.kRotationController.calculate(currentWheelAngleRadians, desiredWheelAngleRadians);        
        double angleMotorRelativeSpeed = SwerveUtils.radiansToRotations(wheelAngleSpeedRadiansPerSecond);
        setAngleMotorRelativeSpeed(angleMotorRelativeSpeed);
        
        double driveMotorRelativeSpeed = SwerveUtils.toDriveRelativeSpeed(wheelDriveSpeedMetersPerSecond);
        setDriveMotorRelativeSpeed(driveMotorRelativeSpeed);

        lastAngleRadians = desiredWheelAngleRadians;
    }

    public double getAngleAbsoluteEncoderPositionRadians() {
        return wheelAngleAbsoluteEncoder.getPositionRadians();
    }

    public double getAngleRelativeEncoderPositionRadians() {
        return SwerveUtils.angleMotorToWheel(angleMotor.getPositionRadians());
    }

    public void setState(SwerveModuleState desiredState) {
        setState(desiredState.speedMetersPerSecond, desiredState.angle.getRadians());
    }

    public SwerveModuleState getState() {
        double speedRadiansPerSecond = SwerveUtils.driveMotorToWheel(driveMotor.getSpeedRadiansPerSecond());
        double speedMetersPerSecond = SwerveUtils.getWheelLinearVelocity(speedRadiansPerSecond);
        Rotation2d angle = Rotation2d.fromRadians(wheelAngleAbsoluteEncoder.getPositionRadians());
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public double getCurrentWheelAngleRadians() {
        switch (SwerveConstants.kEncoderType) {
            case ABSOLUTE:
                return wheelAngleAbsoluteEncoder.getPositionRadians();
            case RELATIVE:
                return SwerveUtils.angleMotorToWheel(angleMotor.getPositionRadians()); // untested
            default:
                return 0;
        }
    }

    public void setDriveMotorRelativeSpeed(double relativeSpeed) {
        driveSpeedEntry.setDouble(relativeSpeed);
        driveMotor.setRelativeSpeed(relativeSpeed);
    }

    public void setAngleMotorRelativeSpeed(double relativeSpeed) {
        angleSpeedEntry.setDouble(relativeSpeed);
        angleMotor.setRelativeSpeed(relativeSpeed);
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        angleMotor.setEncoderPosition(SwerveUtils.angleWheelToMotor(wheelAngleAbsoluteEncoder.getPositionRotations()));
    }

    public SwerveModulePosition getPosition() {
        double distanceMeters = SwerveUtils.driveMotorToWheel(driveMotor.getPositionRadians()) * SwerveConstants.kWheelRadiusMeters;
        Rotation2d angle = Rotation2d.fromRadians(SwerveUtils.angleMotorToWheel(angleMotor.getPositionRadians()));
        return new SwerveModulePosition(distanceMeters, angle);
    }

    public double getAngleDegrees() {
        return Units.rotationsToDegrees(wheelAngleAbsoluteEncoder.getPositionRotations());
    }

    // Print encoder values

    // rotation of wheel
    public void printEncoderPositions(String name) {
        System.out.print(name + ": ");
        double rRot = SwerveUtils.angleMotorToWheel(angleMotor.getPositionRotations());
        double rRad = SwerveUtils.rotationsToRadians(rRot);
        double aRot = wheelAngleAbsoluteEncoder.getPositionRotations();
        double aRad = SwerveUtils.rotationsToRadians(aRot);
        System.out.println("RRot " + rRot + ", RRad " + rRad + ", ARot " + aRot + ", ARad " + aRad);    
    }
    
    public void printDriveEncoderValue(String name) {
        System.out.println(name + ": " + driveMotor.getPositionRotations());
    }
}