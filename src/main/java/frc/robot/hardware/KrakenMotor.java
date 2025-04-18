package frc.robot.hardware;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KrakenMotor {
    private final TalonFX motor;
    
    public KrakenMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        motor = new TalonFX(deviceId, "CANivore2158");

        motor.setNeutralMode(NeutralModeValue.Brake);
        
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(Constants.kAngleCurrentLimit);

        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.apply(feedbackConfigs);
        // configurator.apply(currentLimitsConfigs);
    }

    public double getPositionRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getPositionRadians() {
        return SwerveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        motor.setPosition(position);
    }

    public void setRelativeSpeed(double relativeSpeed) {
        motor.set(relativeSpeed);
    }

    public double getSpeedRotationsPerSecond() {
        return motor.getVelocity().getValueAsDouble();
    }
    public double getSpeedRadiansPerSecond() {
        return SwerveUtils.rotationsToRadians(getSpeedRotationsPerSecond());
    }
} 
