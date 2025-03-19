package frc.robot.hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.SwerveUtils;

public class SparkMaxMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    // private final boolean reverseEncoder;

    public SparkMaxMotor(int deviceId) {
        this(deviceId, false, false);
    }

    public SparkMaxMotor(int deviceId, Boolean reverseMotor) {
        this(deviceId, reverseMotor, false);
    }

    public SparkMaxMotor(int deviceId, Boolean reverseMotor, Boolean isBrake) {
        motor = new SparkMax(deviceId, MotorType.kBrushless);

        SparkBaseConfig config = new SparkMaxConfig();
        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        config.inverted(reverseMotor);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = motor.getEncoder();
    }

    public double getPositionRotations() {
        return encoder.getPosition();
    }
    public double getPositionRadians() {
        return SwerveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public void set(double relativeSpeed) {
        motor.set(relativeSpeed);
    }
}