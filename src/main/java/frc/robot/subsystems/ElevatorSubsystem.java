// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;
import frc.robot.hardware.SparkMaxMotor;

public class ElevatorSubsystem extends SubsystemBase {
	SparkMaxMotor leftElevatorMotor = new SparkMaxMotor(DeviceIds.kLeftElevatorMotor, false, false, true);
	SparkMaxMotor rightElevatorMotor = new SparkMaxMotor(DeviceIds.kRightElevatorMotor, true, false, true);

	PIDController elevatorPIDController = new PIDController(.0001, 0, 0);
    Encoder elevatorEncoder = new Encoder(0, 2);

	double currentPosition = 0;
	IntakeState currentGoal = null;

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {}

	public void stop() {
		leftElevatorMotor.set(0);
		rightElevatorMotor.set(0);
	}

	public boolean atSetpoint(IntakeState intakeState) {
		return Math.abs(currentPosition) >= Math.abs(intakeState.getElevatorValue());
	}

	public void setPosition(IntakeState intakeState) {
		System.out.println("at setpoint??? " + atSetpoint(intakeState));
		currentPosition = elevatorEncoder.getDistance();

		if (!atSetpoint(intakeState)) {
			System.out.println("RUNNING THE MOTORS");
			System.out.println("curr point: " + currentPosition + " whereas my desired position is: " + intakeState.getElevatorValue());
			leftElevatorMotor.set(1);
			rightElevatorMotor.set(1);
		} else {
			if (intakeState.getElevatorValue() != 0) {
				stop();
			} else {
				resetElevatorPosition();
			}
		}
	}

	public void resetElevatorPosition() {
		double setpoint = 0;
        double pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), setpoint);
		System.out.println(-pidOutput);
		leftElevatorMotor.set(-pidOutput);
		rightElevatorMotor.set(-pidOutput);
	}

	public void setGoal(IntakeState intakeState) {
		currentGoal = intakeState;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (currentGoal != null) {
			if (!atSetpoint(currentGoal)) {
				setPosition(currentGoal);
			}
		}
	}
}