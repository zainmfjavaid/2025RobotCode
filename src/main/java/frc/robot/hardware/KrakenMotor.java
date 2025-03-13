package frc.robot.hardware;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.SwerveUtils;

public class KrakenMotor {
    private final TalonFX motor;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;

    public KrakenMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        motor = new TalonFX(deviceId, "CANivore2158");
        motor.setNeutralMode(NeutralModeValue.Brake);
        
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        
        // TO DO: add feedback configs to motor 

        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public KrakenMotor(int deviceId, Boolean reverse) {
        this(deviceId, reverse, reverse);
    }

    public double getPositionRotations() {
        return reverseEncoder ? -motor.getPosition().getValueAsDouble() : motor.getPosition().getValueAsDouble();
    }
    public double getPositionRadians() {
        return SwerveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        motor.setPosition(reverseEncoder ? -position : position);
    }

    public void setRelativeSpeed(double relativeSpeed) {
        motor.set(reverseMotor ? -relativeSpeed : relativeSpeed);
    }

    public double getSpeedRotationsPerSecond() {
        return motor.getVelocity().getValueAsDouble();
    }
    public double getSpeedRadiansPerSecond() {
        return SwerveUtils.rotationsToRadians(getSpeedRotationsPerSecond());
    }
} 
