package frc.robot.hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkMaxMotor {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;
    SparkBaseConfig config = new SparkMaxConfig();

    public SparkMaxMotor(int deviceId) {
        this(deviceId, false, false, false);
    }

    public SparkMaxMotor(int deviceId, boolean isBrake) {
        this(deviceId, false, false, isBrake);
    }

    public SparkMaxMotor(int deviceId, boolean reverseMotor, boolean reverseEncoder) {
        this(deviceId, reverseMotor, reverseEncoder, false);
    }
    
    public SparkMaxMotor(int deviceId, boolean reverseMotor, boolean reverseEncoder, boolean isBrake) {
        motor = new SparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public SparkMaxMotor(int deviceId, boolean reverseMotor, boolean reverseEncoder, boolean isBrake, int currentLimit) {
        motor = new SparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        config.smartCurrentLimit(currentLimit);
        config.limitSwitch.forwardLimitSwitchEnabled(false);
        config.limitSwitch.reverseLimitSwitchEnabled(false);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.reverseMotor = reverseMotor;
        this.reverseEncoder = reverseEncoder;
    }

    public double getPositionRotations() {
        return reverseEncoder ? -encoder.getPosition() : encoder.getPosition();
    }
    public double getPositionRadians() {
        return SwerveUtils.rotationsToRadians(getPositionRotations());
    }
    
    public void setEncoderPosition(double position) {
        encoder.setPosition(reverseEncoder ? -position : position);
    }

    public void set(double relativeSpeed) {
        motor.set(reverseMotor ? -relativeSpeed : relativeSpeed);
    }
}