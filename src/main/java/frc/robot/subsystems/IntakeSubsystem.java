// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.AlternateEncoderConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(DeviceIds.kArmMotor, false, false, true);
    SparkMaxMotor wristMotor = new SparkMaxMotor(DeviceIds.kWristMotor, false, false, true, 100);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(DeviceIds.kRollerMotor);
    
    PIDController armPIDController = new PIDController(0.035, 0, 0);
    PIDController wristPIDController = new PIDController(0.02, 0, 0);

    IntakeState currentGoal = null;

    public IntakeSubsystem() {}

    public void setPosition(IntakeState intakeState) {
        double armPIDOutput = armPIDController.calculate(armMotor.getPositionRotations(), intakeState.getArmPosition());
        double wristPIDOutput = wristPIDController.calculate(wristMotor.getPositionRotations(), intakeState.getWristValue());

        if (armMotor.getPositionRotations() > 2) {
            wristMotor.set(wristPIDOutput);
        }

        armMotor.set(armPIDOutput);
	}

    public boolean atSetpoint(IntakeState intakeState) {
        return Math.abs(armMotor.getPositionRotations()) > (intakeState.getArmPosition() - 1) && Math.abs(armMotor.getPositionRotations()) < (intakeState.getArmPosition() + 1);
    }

    public void setGoal(IntakeState intakeState) {
        currentGoal = intakeState;
    }

    public void runRollerMotors(double speed) {
        rollerMotor.set(speed);
    }

    @Override
    public void periodic() {
        if (currentGoal != null) {
            setPosition(currentGoal);
        }
    }
}
