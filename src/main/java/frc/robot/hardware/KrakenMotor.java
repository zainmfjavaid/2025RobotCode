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
    // private final boolean reverseEncoder; - MAY BE USED

    public KrakenMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        motor = new TalonFX(deviceId, "CANivore2158");

        motor.setNeutralMode(NeutralModeValue.Brake);
        
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        motor.getConfigurator().apply(feedbackConfigs);

        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public double getPositionRotations() {
        double positionRotations = motor.getPosition().getValueAsDouble();
        return reverseEncoder ? -positionRotations : positionRotations;
    }
    public double getPositionRadians() {
        return SwerveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        motor.setPosition(position);
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
