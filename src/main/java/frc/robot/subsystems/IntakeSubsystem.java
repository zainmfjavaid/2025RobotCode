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

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(32);
    SparkMaxMotor wristMotor = new SparkMaxMotor(33);
	SparkMaxMotor kickerMotor = new SparkMaxMotor(31, false, false, true);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(30);

    Encoder armEncoder = new Encoder(2, 3);
    Encoder wristEncoder = new Encoder(4, 5);
    
    PIDController armPIDController = new PIDController(0.001, 0, 0);
    PIDController wristPIDController = new PIDController(0.001, 0, 0);

    IntakeState currentGoal = null;

    public IntakeSubsystem() {}

	public void setWristPosition(double position) {
		wristMotor.set(wristPIDController.calculate(wristMotor.getPositionRotations(), position));
        // TODO: Use the separate bore encoder if relevant
	}

    public void setArmPosition(double position) {
        armMotor.set(armPIDController.calculate(armMotor.getPositionRotations(), position));
        // TODO: Still not using the armEncoder
    }

    public void runKickerWheel(double speed) {
        kickerMotor.set(speed);
    }

    public void runRollerMotor(double speed) {
        rollerMotor.set(speed);
    }

    public boolean atSetpoint() {
        double currentWristGoal = currentGoal.getWristValue();
        double currentArmGoal = currentGoal.getArmPosition();
    
        // TODO: SWAP WITH ACTUAL ENCODERS
        double currentWristPosition = wristMotor.getPositionRotations();
        double currentArmPosition = armMotor.getPositionRotations();
    
        double tolerance = 15.0;
    
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
        return new StartEndCommand(() -> {armMotor.set(0.4); System.out.println(armEncoder.getDistance());}, () -> armMotor.set(0), this);
    }

    public StartEndCommand reverseArmTest(){
        return new StartEndCommand(() -> {armMotor.set(-0.4); System.out.println(armEncoder.getDistance());}, () -> armMotor.set(0), this);
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
            if (!atSetpoint()) { // UNCOMMENT THIS 
                // setArmPosition(currentGoal.getArmPosition());
                // setWristPosition(currentGoal.getWristValue());
            }
        }
    }
}
