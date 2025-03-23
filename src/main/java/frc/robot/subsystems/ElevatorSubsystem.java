// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;
import frc.robot.hardware.SparkMaxMotor;

public class ElevatorSubsystem extends SubsystemBase {
	private final SparkMaxMotor leftElevatorMotor = new SparkMaxMotor(DeviceIds.kLeftElevatorMotor, false, true);
	private final SparkMaxMotor rightElevatorMotor = new SparkMaxMotor(DeviceIds.kRightElevatorMotor, true, true);

	PIDController elevatorPIDController = new PIDController(0.0003, 0, 0);

    Encoder elevatorEncoder = new Encoder(0, 1);
	DigitalInput limitSwitch = new DigitalInput(9);

	double currentPosition = 0;
	IntakeState currentGoal = null;
	boolean isDown = false;
	boolean isOverride = false;

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {}

	public void setSpeed(double speed) {
		leftElevatorMotor.set(speed);
		rightElevatorMotor.set(speed);
	}

	public void stop() {
		setSpeed(0);
	}

	public void goDown() {
		setSpeed(-0.4);
	}

	public void goUp() {
		setSpeed(0.4);
	}

	public void setPosition(IntakeState intakeState) {
		double pidOutput;
		if (isDown) {
			if (limitSwitch.get()) {
				goDown();
			} else {
				stop();
			}
		} else {
			pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), intakeState.getElevatorValue());
			setSpeed(-pidOutput);
		}
	}

	public void setGoal(IntakeState intakeState) {
		if (currentGoal != null && currentGoal.getElevatorValue() < intakeState.getElevatorValue()) {
			isDown = true;
		} else {
			isDown = false;
		}
		currentGoal = intakeState;
	}

	public boolean atSetpoint() {
		System.out.println(elevatorEncoder.getDistance());
		if (Math.abs(currentGoal.getElevatorValue() - elevatorEncoder.getDistance()) < 300) {
			System.out.println("AT SETPOINT");
			return true;
		}

		return false;
	}

	public void setOverride(boolean newOverrideValue) {
		isOverride = newOverrideValue;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (currentGoal != null && !isOverride) {
			setPosition(currentGoal);
		}
	}
}
