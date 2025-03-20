package frc.robot.hardware;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.DriveUtils;

public class KrakenMotor {
    private final TalonFX motor;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;

    public KrakenMotor(int deviceId, boolean isBrake, boolean reverseMotor, boolean reverseEncoder) {
        motor = new TalonFX(deviceId, "CANivore2158");
        motor.setNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public KrakenMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        this(deviceId, true, reverseMotor, reverseEncoder);
    }

    public KrakenMotor(int deviceId, Boolean reverse) {
        this(deviceId, reverse, reverse);
    }

    // getters
    public double getPositionRotations() {
        double position = motor.getPosition().getValueAsDouble();
        return reverseEncoder ? -position : position;
    }

    public double getPositionRadians() {
        return DriveUtils.rotationsToRadians(getPositionRotations());
    }

    public double getSpeedRotationsPerSecond() {
        return motor.getVelocity().getValueAsDouble();
    }
    public double getSpeedRadiansPerSecond() {
        return DriveUtils.rotationsToRadians(getSpeedRotationsPerSecond());
    }

    // setters
    public void setEncoderPosition(double position) {
        motor.setPosition(reverseEncoder ? -position : position);
    }

    public void setRelativeSpeed(double relativeSpeed) {
        motor.set(reverseMotor ? -relativeSpeed : relativeSpeed);
    }

    public void setIdleMode(boolean isBrake) {
        motor.setNeutralMode(isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
} 
