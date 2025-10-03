// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.SystemSpeeds;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkFlex armMotor = new SparkFlex(DeviceIds.kArmMotor, MotorType.kBrushless);
    SparkMax wristMotor = new SparkMax(DeviceIds.kWristMotor, MotorType.kBrushless);
    SparkFlex rollerMotor = new SparkFlex(DeviceIds.kRollerMotor, MotorType.kBrushless);

    // SparkAbsoluteEncoder armMotorEncoder = armMotor.getAbsoluteEncoder();
    // SparkAbsoluteEncoder wristMotorEncoder = wristMotor.getAbsoluteEncoder();

    CANcoder armCANcoder = new CANcoder(5, "rio");
    CANcoder wristCANcoder = new CANcoder(6, "rio");
    
    // CanCoder 5 arm rotation (Absolute Position Offset: 1.456055rev)
    // CanCoder 6 wrist rotation

    // arm -1.966 bottom  -23.382 top

    // starting 2.556


    // 0.4556

    PIDController armPIDController = new PIDController(2, 0, 0.1);
    PIDController wristPIDController = new PIDController(1.1, 0, 0.07);

    IntakeState currentGoal = IntakeState.STOW;
    LaserCan lc = new LaserCan(22);

    public IntakeSubsystem() {
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        wristConfig.apply(new SoftLimitConfig()
            .forwardSoftLimit(18)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
        );

        
        // IMPORTANT: starting arm and wrist position should be in stow

        SparkFlexConfig armConfig = new SparkFlexConfig();
        armConfig.apply(new SoftLimitConfig()
            .forwardSoftLimit(20.48) // 19.33
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true)
        );

        
        CANcoderConfiguration config = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withMagnetOffset(0.4556));
        armCANcoder.getConfigurator().apply(config);

        // SparkFlexConfig armConfig = new SparkFlexConfig();
        // armConfig
        // .apply(
        // new SoftLimitConfig()
        // .forwardSoftLimit(0)
        // .reverseSoftLimit(-19)
        // .forwardSoftLimitEnabled(true)
        // .reverseSoftLimitEnabled(true)
        // );
            
        // .idleMode(IdleMode.kBrake); // .inverted(true);

        // wristMotor.configure(wristConfig, null, null);
        armMotor.configure(armConfig, null, null);

        wristPIDController.enableContinuousInput(-0.5, 0.5);
    }

    public double getDistance() {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm;
        } else {
            return 999.9;
        }
    }

    public boolean getHasMeasurement() {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null) {
            return measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
        } else {
            return false;
        }
    }

    public void setPosition(IntakeState intakeState) {
        armPIDController.setTolerance(0.03);
        wristPIDController.setTolerance(0.05);
        
        double armPIDOutput = armPIDController.calculate(
            getArmPosition(), intakeState.getArmPosition());
        double wristPIDOutput = wristPIDController.calculate(
            getWristPosition(), intakeState.getWristValue());
        
        armMotor.set(0);
        wristMotor.set(0);

        while (!(armPIDController.atSetpoint() && wristPIDController.atSetpoint())){
            MathUtil.clamp(wristPIDOutput, -1, 1);
            MathUtil.clamp(armPIDOutput, -1, 1);

            wristPIDOutput = wristPIDController.calculate(
                getWristPosition(), intakeState.getWristValue());
            armPIDOutput = armPIDController.calculate(
                getArmPosition(), intakeState.getArmPosition());
            
            System.out.println("W: " + getWristPosition() + "\n" + "A: " + getArmPosition() + "\n" + "W-Out:" + wristPIDOutput);
            System.out.println(wristPIDController.getSetpoint());

            wristMotor.set(wristPIDOutput);
            
            armMotor.set(armPIDOutput);
            // System.out.println(armMotor.getEncoder().getVelocity());
        }
	}

    final double atSetpointThreshold = 0.01; // rotations

    public boolean atSetpoint() {
        return armPIDController.atSetpoint() 
        // && wristPIDController.atSetpoint()
        ;
    }

    public double getArmPosition() {
        return armCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getWristPosition() {
        return wristCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setGoal(IntakeState intakeState) {
        currentGoal = intakeState;
    }

    public IntakeState getGoal(){
        return currentGoal;
    }

    public void runRollerMotors(double speed) {
        // if (getDistance() <= 45 && speed >= 0) {
        //     rollerMotor.set(speed);
        // } else {
        //     rollerMotor.set(speed);
        // }
        rollerMotor.set(speed);
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    public StartEndCommand outtakeCommand() {
        return new StartEndCommand(() -> runRollerMotors(SystemSpeeds.kOuttakeRollerSpeed), () -> runRollerMotors(0), this);
    }

    public StartEndCommand intakeCommand() {
        return new StartEndCommand(() -> runRollerMotors(0.6), () -> runRollerMotors(0), this);
    }

    public StartEndCommand runWristForward() {
        return new StartEndCommand(() -> wristMotor.set(0.1), () -> stopWristMotor(), this);
    }

    public StartEndCommand runWristBackward() {
        return new StartEndCommand(() -> wristMotor.set(-0.1), () -> stopWristMotor(), this);
    }

    public StartEndCommand runArmForward() {
        return new StartEndCommand(() -> armMotor.set(0.5), () -> armMotor.set(0), this);
    }

    public StartEndCommand runArmBackward() {
        return new StartEndCommand(() -> armMotor.set(-0.5), () -> armMotor.set(0), this);
    }

    public InstantCommand printEncoderValues() {
        return new InstantCommand(() -> {System.out.println("W: " + wristCANcoder.getAbsolutePosition().getValueAsDouble() + "\nA: " + armCANcoder.getAbsolutePosition().getValueAsDouble());}, this);
    }

    public boolean intakeHasCoral() {
        return getDistance() == 0.0 && getHasMeasurement();
    }


    public void printWrist() {
        System.out.println(wristCANcoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        //System.out.println("Lasercan reading:  " + getDistance());
        // if (currentGoal != null) {
        //     setPosition(currentGoal);
        // }
    }
}
