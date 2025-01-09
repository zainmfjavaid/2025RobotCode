package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.IntakeConstants;

// make sure the intake deploy motor is retracted when starting

// FUTURE UPDATES
// take into account force from gravity
// to make the deploy and retract smoother
// this should similar to pendulum, so there is harmonic motion (sinusoidal)

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeDeployMotor = new SparkMax(MotorConstants.kIntakeDeployMotorDeviceId, MotorType.kBrushless);
    private final RelativeEncoder intakeDeployMotorRelativeEncoder = intakeDeployMotor.getEncoder();
    
    private final SparkMax intakeRollerMotor = new SparkMax(MotorConstants.kIntakeRollerMotorDeviceId, MotorType.kBrushless);
    private final SparkMax intakeIndexMotor = new SparkMax(MotorConstants.kIntakeIndexMotorDeviceId, MotorType.kBrushless);
    
    private final SparkMax intakeShooterInMotor = new SparkMax(11, MotorType.kBrushless);

    /*
     * Return the relative encoder position of the intake deploy motor
     * @return position in rotations
     */
    public double getIntakeDeployRelativePosition() {
        return intakeDeployMotorRelativeEncoder.getPosition();
    }

    private double positionToSpeed(double errorPosition) {
        return errorPosition * IntakeConstants.kP;
    }

    public void setIntakeDeployMotor(double desiredPosition) {
        double currentPosition = getIntakeDeployRelativePosition();
        double errorPosition = desiredPosition - currentPosition;
        double speed = positionToSpeed(errorPosition);
        intakeDeployMotor.set(speed);
    }

    public void setIntakeRollerMotor(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setIntakeIndexMotor(double speed) {
        intakeIndexMotor.set(speed);
    }

    public void setIntakeShooterInMotor(double speed) {
        intakeShooterInMotor.set(speed);
    }
}
