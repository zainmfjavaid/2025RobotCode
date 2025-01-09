package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModuleKraken {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final double P = 0.2;
    private final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(14);

    public SwerveModuleKraken(int driveMotorDeviceId, int turnMotorDeviceId) {
        driveMotor = new TalonFX(driveMotorDeviceId);
        turnMotor = new TalonFX(turnMotorDeviceId);
    }

    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    public TalonFX getTurnMotor() {
        return turnMotor;
    }

    public void setState(SwerveModuleState state) {
        double speedMetersPerSecond = state.speedMetersPerSecond;
        driveMotor.set(speedMetersPerSecond / MAX_SPEED_METERS_PER_SECOND);
        setAngle(state.angle);
    }

    public void setAngle(Rotation2d desiredAngle) {
        Rotation2d currentAngle = Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble());
        double errorRadians = desiredAngle.getRadians() - currentAngle.getRadians();
        if (Math.abs(errorRadians) < Math.PI) {
            errorRadians = -errorRadians;
        }
        double speed = errorRadians * P;
        turnMotor.set(speed);
    }
}