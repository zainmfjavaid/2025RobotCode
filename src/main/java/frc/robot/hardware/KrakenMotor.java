package frc.robot.hardware;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.SwerveUtils;

public class KrakenMotor {
    private final TalonFX motor;

    public KrakenMotor(int deviceId, boolean inverted) {
        motor = new TalonFX(deviceId, "CANivore2158");

        motor.setNeutralMode(NeutralModeValue.Brake);
        
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        motor.setInverted(inverted);

        // MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        //     .withInverted(InvertedValue.CounterClockwise_Positive);

        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.apply(feedbackConfigs);
        // configurator.apply(motorOutputConfigs);
    }

    public double getPositionRotations() {
        return motor.getRotorPosition().getValueAsDouble();
    }
    public double getPositionRadians() {
        return SwerveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        motor.setPosition(position);
    }

    // Duty Cycle
    public void setRelativeSpeed(double relativeSpeed) {
        motor.set(relativeSpeed);
    }
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    public double getMotorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }
    public double getSupplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    public double getSpeedRotationsPerSecond() {
        return motor.getVelocity().getValueAsDouble();
    }
    public double getSpeedRadiansPerSecond() {
        return SwerveUtils.rotationsToRadians(getSpeedRotationsPerSecond());
    }
} 
