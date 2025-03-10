// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(DeviceIds.kArmMotor);
    SparkMaxMotor wristMotor = new SparkMaxMotor(DeviceIds.kWristMotor);
	SparkMaxMotor kickerMotor = new SparkMaxMotor(DeviceIds.kKickerMotor, false, false, true);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(DeviceIds.kRollerMotor);

    Encoder armEncoder = new Encoder(2, 3);
    Encoder wristEncoder = new Encoder(4, 5);
    
    PIDController armPIDController = new PIDController(0.06, 0, 0);
    PIDController armPIDUpController = new PIDController(0.15, 0, 0);

    PIDController wristPIDController = new PIDController(0.001, 0, 0);

    IntakeState currentGoal = null;

    public IntakeSubsystem() {}

	public void setWristPosition(double position) {
		wristMotor.set(wristPIDController.calculate(wristMotor.getPositionRotations(), position));
        // TODO: Use the separate bore encoder if relevant
	}

    public void setArmPosition(double position, boolean isUp) {
        if (!isUp) {
            System.out.println(armPIDController.calculate(armMotor.getPositionRotations(), position));
            System.out.println("Currently at: " + armMotor.getPositionRotations() + " whereas i am trying to go to: " + position);
            armMotor.set(armPIDController.calculate(armMotor.getPositionRotations(), position));
        } else {
            System.out.println("MOVING UP " + armPIDController.calculate(armMotor.getPositionRotations(), position));
            System.out.println("j");
            // armMotor.set(armPIDUpController.calculate(armMotor.getPositionRotations(), position));
        }
        // TODO: Still not using the armEncoder
    }

    public void runKickerWheel(double speed) {
        kickerMotor.set(speed);
    }

    public void runRollerMotor(double speed) {
        rollerMotor.set(speed);
    }

    public void resetArmEncoder() {
        armMotor.setEncoderPosition(0);
    }

    public boolean atSetpoint() {
        double currentWristGoal = currentGoal.getWristValue();
        double currentArmGoal = currentGoal.getArmPosition();
    
        // TODO: SWAP WITH ACTUAL ENCODERS
        double currentWristPosition = wristMotor.getPositionRotations();
        double currentArmPosition = armMotor.getPositionRotations();
    
        double tolerance = 1.0;
        
        return Math.abs(currentWristPosition - currentWristGoal) <= tolerance &&
               Math.abs(currentArmPosition - currentArmGoal) <= tolerance;
    }    

    public void setGoal(IntakeState intakeState) {
        currentGoal = intakeState;
    }

    // Test commands
    public StartEndCommand runRollersTest() {
        return new StartEndCommand(() -> runRollerMotor(0.5), () -> runRollerMotor(0), this);
    }

    public StartEndCommand reverseRollersTest() {
        return new StartEndCommand(() -> runRollerMotor(-0.3), () -> runRollerMotor(0), this);
    }

    public StartEndCommand runArmTest(){
        return new StartEndCommand(() -> {armMotor.set(0.4); System.out.println(armMotor.getPositionRotations());}, () -> armMotor.set(0), this);
    }

    public StartEndCommand reverseArmTest(){
        return new StartEndCommand(() -> {armMotor.set(-0.4); System.out.println(armMotor.getPositionRotations());}, () -> armMotor.set(0), this);
    }

    public StartEndCommand runWristTest() {
        return new StartEndCommand(() -> {wristMotor.set(0.1); System.out.println(wristEncoder.getDistance());}, () -> {wristMotor.set(0);}, this);
    }

    public StartEndCommand reverseWristTest() {
        return new StartEndCommand(() -> {wristMotor.set(-0.1); System.out.println(wristEncoder.getDistance());}, () -> wristMotor.set(0), this);
    }

    public StartEndCommand runKickerTest() {
        return new StartEndCommand(() -> runKickerWheel(0.5), () -> runKickerWheel(0), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (currentGoal != null) {
            System.out.println(currentGoal);
            if (!atSetpoint()) { // UNCOMMENT THIS
                setArmPosition(currentGoal.getArmPosition(), false);
                // setWristPosition(currentGoal.getWristValue());
            } else {
                resetArmEncoder();
            }
        }
    }
}
