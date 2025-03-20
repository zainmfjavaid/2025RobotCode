// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.KrakenMotor;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;

public class SwerveModule extends SubsystemBase {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(1.5); // Motor speed when set to 1.0
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
    public static final double kDriveMotorGearRatio = 1.0 / 5.0; // (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kTurningMotorGearRatio = 1.0 / 3.0; // (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kDriveEncoderRotFactor = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; //Conversion factor converting the Drive Encoder's rotations to meters
    public static final double kDriveEncoderRPMFactor = kDriveEncoderRotFactor / 60; //Conversion factor converting the Drive Encoder's RPM to meters per second
    public static final double kTurningEncoderRotFactor = kTurningMotorGearRatio * 2 * Math.PI; //Conversion factor converting the Turn Encoder's rotations to Radians
    public static final double kTurningEncoderRPMFactor = kTurningEncoderRotFactor / 60; //Conersion factor converting the Turn Encoder's RPM to radians per second
    public static final double kPTurning = 0.025; //P gain for the turning motor

    private final KrakenMotor driveMotor;
    private final KrakenMotor turningMotor;

    private final PIDController turningPIDController;
    private double turningSetpoint;

    private final CANcoder absoluteEncoder;
    private final EncoderConfig absoluteEncoderConfig;

    private final String ID;

    public SwerveModule(MotorConfig driveMotorConfig, MotorConfig turningMotorConfig, EncoderConfig absoluteEncoderConfig, String ID) {

        this.ID = ID;

        absoluteEncoder = AbsoluteEncoder.constructCANCoder(absoluteEncoderConfig);
        this.absoluteEncoderConfig = absoluteEncoderConfig;

        driveMotor = MotorController.constructMotor(driveMotorConfig);
        turningMotor = MotorController.constructMotor(turningMotorConfig);

        // NOTE: REMOVED SINCE NOT A THING IN KRAKEN. COULD CAUSE ISSUES?
        // driveEncoder.setPositionConversionFactor(kDriveEncoderRotFactor);
        // driveEncoder.setVelocityConversionFactor(kDriveEncoderRPMFactor);
        // turningEncoder.setPositionConversionFactor(kTurningEncoderRotFactor);
        // turningEncoder.setVelocityConversionFactor(kTurningEncoderRPMFactor);
        
        turningPIDController = new PIDController(kPTurning, 0, 0);

        resetEncoders();
    }

    public String getID() {
        return ID;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public double getDrivePosition() {
        return driveMotor.getPositionRotations() * kDriveEncoderRotFactor; // Convert to meters
    }
    
    public double getDriveVelocity() {
        return driveMotor.getSpeedRotationsPerSecond() * kDriveEncoderRotFactor;
    }

    public double getTurningPosition() {
        //The math for this remainder is position - (2pi * Math.round(position/2pi))
        return Math.IEEEremainder(turningMotor.getPositionRadians(), 2 * Math.PI);
    }

    public double getTurningVelocity() {
        return turningMotor.getSpeedRotationsPerSecond() * kTurningEncoderRotFactor; // Convert to radians per second
    }

    public double getAbsoluteTurningPosition() { 
        return Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetEncoders() {
        driveMotor.setEncoderPosition(0);
        System.out.println("RESETRESETRESET");
        System.out.println(getAbsoluteTurningPosition());
        turningMotor.setEncoderPosition(Units.radiansToRotations(getAbsoluteTurningPosition()));
        if (Robot.isReal()) {
            absoluteEncoder.close();
        }
    }    

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public double wrapEncoderValue(double encoderValue) { // chatgpt method
        // Wrap the encoder value between 0 and 2π
        return ((encoderValue % (Math.PI * 2)) + (Math.PI * 2)) % (Math.PI * 2);
    }

    private double calculateSetpoint(double stateAngle){
        // System.out.println(Units.radiansToDegrees(turningMotor.getPositionRadians() + Math.IEEEremainder(stateAngle - getTurningPosition(), 2 * Math.PI)));
        return wrapEncoderValue(turningMotor.getPositionRadians() + Math.IEEEremainder(stateAngle - getTurningPosition(), 2 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) == 0) { //Prevent wheels to returning to heading of 0 when controls released
            stop();
            return;
        }
        
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        driveMotor.setRelativeSpeed(desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond);
        turningSetpoint = calculateSetpoint(desiredState.angle.getRadians());

        turningMotor.setRelativeSpeed(turningPIDController.calculate(wrapEncoderValue(turningMotor.getPositionRadians() / 4), turningSetpoint));

        if (ID.equals("BL")) System.out.println("I AM " + ID + " and im trying to go to " + Units.radiansToDegrees(turningSetpoint) + " but sadly im still at " + Units.radiansToDegrees(turningMotor.getPositionRadians() / 4) + " (power " + turningPIDController.calculate(turningMotor.getPositionRadians() / 4, turningSetpoint) + ")");
        // turningPIDController.setReference(turningSetpoint, ControlType.kPosition); - THIS DOES NOT WORK WITH NEW API. CORRECT LOGIC

        SmartDashboard.putString("Swerve[" + ID + "] state", desiredState.toString());
    }

    public void park(boolean reversed) {
        stop();
        double targetAngle = reversed ? Math.PI / 4 : -Math.PI / 4; // Target angle in radians
        turningMotor.setRelativeSpeed(turningPIDController.calculate(turningMotor.getPositionRadians(), targetAngle));
    }    

    public void stop() {
        driveMotor.setRelativeSpeed(0);
        turningMotor.setRelativeSpeed(0);
    }

    public void coast() {
        driveMotor.setIdleMode(false);
        turningMotor.setIdleMode(false);
    }

    public void brake() {
        driveMotor.setIdleMode(true);
        turningMotor.setIdleMode(true);
    }

    public String getStatusAbsoulteEncoder() {
        double offset = absoluteEncoderConfig.getOffset();
        return String.format("%s: %.4f(rad) [off: %.4f] ", ID, getAbsoluteTurningPosition(), offset);
    }

    @Override
    public void periodic(){
        // if (ID.equals("BL")) System.out.println("I AM " + ID + " and im trying to go to " + Units.radiansToDegrees(turningSetpoint) + " but sadly im still at " + Units.radiansToDegrees(turningMotor.getPositionRadians() / 4));
    }
}