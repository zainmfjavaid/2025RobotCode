// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ElevatorSubsystem extends SubsystemBase {
	private final SparkMaxMotor leftElevatorMotor = new SparkMaxMotor(DeviceIds.kLeftElevatorMotor, false, true, true);
	private final SparkMaxMotor rightElevatorMotor = new SparkMaxMotor(DeviceIds.kRightElevatorMotor, true, true, true);

	PIDController elevatorPIDController = new PIDController(0.0003, 0, 0);
	PIDController elevatorDownPIDController = new PIDController(0.0004, 0.0003, 0);

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
		setSpeed(0.4);
	}

	public void setPosition(IntakeState intakeState) {
		double pidOutput;
		double downPidOutput;
		// if (intakeState.getElevatorValue() == 0) {
		// 	if (limitSwitch.get()) {
		// 		goDown();
		// 	} else {
		// 		stop();
		// 	}
		// } else {
			// pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), intakeState.getElevatorValue());
			// setSpeed(pidOutput);
		// }
		pidOutput = elevatorPIDController.calculate(elevatorEncoder.getDistance(), intakeState.getElevatorValue());
		downPidOutput = elevatorDownPIDController.calculate(elevatorEncoder.getDistance(), intakeState.getElevatorValue());

		if ((!limitSwitch.get() || (Math.abs(elevatorEncoder.getDistance()) < 30)) && pidOutput > 0) {
			stop();
		} else if (pidOutput > 0){
			setSpeed(downPidOutput);
		} else {
			setSpeed(pidOutput);
		}
	}

	public void setGoal(IntakeState intakeState) {
		if ((currentGoal != null && currentGoal.getElevatorValue() < intakeState.getElevatorValue()) || intakeState == IntakeState.SCORESTOW) {
			isDown = true;
		} else {
			isDown = false;
		}
		currentGoal = intakeState;
		// System.out.println(currentGoal);
		// if (currentGoal == IntakeState.L4 || currentGoal == IntakeState.L3 || currentGoal == IntakeState.L2) {
		// 	SwerveSubsystem.driveSpeedConstant = 0.2;
		// 	SwerveSubsystem.angleSpeedConstant = 0.4;
		// 	// SwerveSubsystem.isFieldRelative = false;
		// } else {
		// 	SwerveSubsystem.driveSpeedConstant = 1;
		// 	SwerveSubsystem.angleSpeedConstant = 1;
		// 	// SwerveSubsystem.isFieldRelative = true;
		// }
	}

	public boolean atSetpoint() {
		if (Math.abs(currentGoal.getElevatorValue() - elevatorEncoder.getDistance()) < 500) {
			return true;
		}

		return false;
	}

	public boolean atAnySpecificGeneralSetpointFeaturingKennysonLe(IntakeState intakeState) {
		if (Math.abs(intakeState.getElevatorValue() - elevatorEncoder.getDistance()) < 500) {
			return true;
		}

		return false;
	}

	public void setOverride(boolean newOverrideValue) {
		isOverride = newOverrideValue;
	}

	public StartEndCommand runElevatorTest() {
		return new StartEndCommand(() -> setSpeed(0.2), () -> setSpeed(0), this);
	}

	public StartEndCommand reverseElevatorTest() {
		return new StartEndCommand(() -> setSpeed(-0.6), () -> setSpeed(0), this);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (currentGoal != null && !isOverride) {
			setPosition(currentGoal);
		}
	}
}
