// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.Constants.IntakeConstants.IntakeState;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(15);
    Servo wrist = new Servo(0);
	SparkMaxMotor kickerMotor = new SparkMaxMotor(7);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(1);
    
    PIDController armPIDController = new PIDController(0.001, 0, 0);

    public IntakeSubsystem() {}

	public void setWristAngle(double angle) {
		wrist.setAngle(angle);
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
    
	public InstantCommand rotateWristTest() {
		return new InstantCommand(() -> {
			wrist.setAngle(45);
		}, this);
	}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
