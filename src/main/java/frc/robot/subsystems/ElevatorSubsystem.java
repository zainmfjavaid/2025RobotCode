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

	PIDController elevatorPIDController = new PIDController(0.01, 0, 0);
    Encoder elevatorEncoder = new Encoder(0, 1);

	double currentPosition = 0;
	IntakeState currentGoal = null;

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {}

	public void stop() {
		leftElevatorMotor.set(0);
		rightElevatorMotor.set(0);
	}

	public void goDown() {
		leftElevatorMotor.set(-0.2);
		rightElevatorMotor.set(-0.2);
	}

	public void goUp() {
		leftElevatorMotor.set(0.4);
		rightElevatorMotor.set(0.4);
	}

	public void setPosition(IntakeState intakeState) {
		double pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), intakeState.getElevatorValue());
		System.out.println(pidOutput);

		//leftElevatorMotor.set(pidOutput);
		//rightElevatorMotor.set(pidOutput);
	}

	public void resetElevatorPosition() {
        double pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), 0);

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
			setPosition(currentGoal);
		}
	}
}
