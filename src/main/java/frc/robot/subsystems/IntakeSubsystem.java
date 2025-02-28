// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(32);
    Servo wrist = new Servo(9);
	SparkMaxMotor kickerMotor = new SparkMaxMotor(31, false, false, true);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(30);

    Encoder armEncoder = new Encoder(2, 3);


    int cycles = 0;
    
    PIDController armPIDController = new PIDController(0.001, 0, 0);

    public IntakeSubsystem() {}

	public void setWristAngle(double angle) {
        System.out.println("runnong");
		wrist.set(angle);
	}

    public void setArmPosition(double position) {
        armMotor.set(armPIDController.calculate(armMotor.getPositionRotations(), position));
    }

    public void runKickerWheel(double speed) {
        kickerMotor.set(speed);
    }

    public void setIntakeState(IntakeState intakeState) {
        double wristAngle = intakeState.getWristAngle();
        double armPosition = intakeState.getArmPosition();

        setArmPosition(armPosition);
        setWristAngle(wristAngle);
    }

    public void runRollerMotor(double speed) {
        rollerMotor.set(speed);
    }

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

    public StartEndCommand runKickerTest() {
        return new StartEndCommand(() -> runKickerWheel(0.5), () -> runKickerWheel(0), this);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
