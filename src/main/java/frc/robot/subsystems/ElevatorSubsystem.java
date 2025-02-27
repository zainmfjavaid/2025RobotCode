// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.hardware.SparkMaxMotor;

public class ElevatorSubsystem extends SubsystemBase {
	SparkMaxMotor leftElevatorMotor = new SparkMaxMotor(5, false, false, true);
	SparkMaxMotor rightElevatorMotor = new SparkMaxMotor(6, true, false, true);

	PIDController elevatorPIDController = new PIDController(0.1, 0, 0);
    Encoder elevatorEncoder = new Encoder(0, 1);

	/** Creates a new ElevatorSubsystem. */
	public ElevatorSubsystem() {}

	public void stop() {
		leftElevatorMotor.set(0);
		rightElevatorMotor.set(0);
	}

	public void setPosition(IntakeState intakeState) {
		double setpoint = intakeState.getElevatorValue();
        double pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), setpoint);

		leftElevatorMotor.set(pidOutput);
		rightElevatorMotor.set(pidOutput);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}