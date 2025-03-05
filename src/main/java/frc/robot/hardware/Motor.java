package frc.robot.hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.subsystems.DriveUtils;

<<<<<<<< HEAD:src/main/java/frc/robot/hardware/Motor.java
public class Motor {
========
public class SparkMaxMotor {
>>>>>>>> 813de6ede1abfbe03e33dbac5ea31f58e728451a:src/main/java/frc/robot/hardware/SparkMaxMotor.java
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final boolean reverseMotor;
    private final boolean reverseEncoder;
    SparkBaseConfig config = new SparkMaxConfig();

    public SparkMaxMotor(int deviceId) {
        this(deviceId, false, false, false);
    }

    public SparkMaxMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder) {
        this(deviceId, reverseMotor, reverseEncoder, false);
    }
    
    public SparkMaxMotor(int deviceId, Boolean reverseMotor, Boolean reverseEncoder, Boolean isBrake) {
        motor = new SparkMax(deviceId, MotorType.kBrushless);
        encoder = motor.getEncoder();

        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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