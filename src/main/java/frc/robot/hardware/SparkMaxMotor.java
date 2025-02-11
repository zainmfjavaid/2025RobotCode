package frc.robot.hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.subsystems.DriveUtils;

public class SparkMaxMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;

    public SparkMaxMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        motor = new SparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();
        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public double getPositionRotations() {
        return reverseEncoder ? -encoder.getPosition() : encoder.getPosition();
    }
    public double getPositionRadians() {
        return DriveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        encoder.setPosition(reverseEncoder ? -position : position);
    }

    public void set(double relativeSpeed) {
        motor.set(reverseMotor ? -relativeSpeed : relativeSpeed);
    }
}