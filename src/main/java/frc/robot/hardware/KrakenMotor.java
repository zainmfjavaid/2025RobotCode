package frc.robot.hardware;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.DriveUtils;

public class KrakenMotor {
    private final TalonFX motor;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;

    public KrakenMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        motor = new TalonFX(deviceId, "CANivore2158");
        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public double getPositionRotations() {
        double positionRotations = motor.getPosition().getValueAsDouble();
        return reverseEncoder ? -positionRotations : positionRotations;
    }

    public double getPositionRadians() {
        return DriveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        motor.setPosition(reverseEncoder ? -position : position);
    }

    public void set(double relativeSpeed) {
        motor.set(reverseMotor ? -relativeSpeed : relativeSpeed);
    }
}
