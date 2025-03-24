// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.SystemSpeeds;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(DeviceIds.kArmMotor, true);
    SparkMaxMotor wristMotor = new SparkMaxMotor(DeviceIds.kWristMotor, true);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(DeviceIds.kRollerMotor);
    
    PIDController armPIDController = new PIDController(0.035, 0, 0);
    PIDController wristPIDController = new PIDController(0.02, 0, 0);

    IntakeState currentGoal = null;
    LaserCan lc = new LaserCan(22);

    public IntakeSubsystem() {}

    public double getDistance() {
        return lc.getMeasurement().distance_mm;
    }

    public void setPosition(IntakeState intakeState) {
        double armPIDOutput = armPIDController.calculate(armMotor.getPositionRotations(), intakeState.getArmPosition());
        double wristPIDOutput = wristPIDController.calculate(wristMotor.getPositionRotations(), intakeState.getWristValue());

        if (armMotor.getPositionRotations() > 2) {
            wristMotor.set(wristPIDOutput);
        } else {
            wristMotor.set(0);
        }

        armMotor.set(armPIDOutput);
	}

    public boolean atSetpoint(IntakeState intakeState) {
        return (Math.abs(armMotor.getPositionRotations()) > (intakeState.getArmPosition() - 1) 
        && Math.abs(armMotor.getPositionRotations()) < (intakeState.getArmPosition() + 1)) 
        && (Math.abs(wristMotor.getPositionRotations()) > (intakeState.getWristValue() - 4) 
        && Math.abs(wristMotor.getPositionRotations()) < (intakeState.getWristValue() + 4));
    }

    public void setGoal(IntakeState intakeState) {
        currentGoal = intakeState;
    }

    public void runRollerMotors(double speed) {
        if (getDistance() <= 45 && speed >= 0) {
            rollerMotor.set(speed);
        } else {
            rollerMotor.set(speed);
        }
    }

    public void stopWristMotor() {
        wristMotor.set(0);
    }

    public StartEndCommand outtakeCommand() {
        return new StartEndCommand(() -> runRollerMotors(SystemSpeeds.kOuttakeRollerSpeed), () -> runRollerMotors(0), this);
    }

    @Override
    public void periodic() {
        if (currentGoal != null) {
            setPosition(currentGoal);
        }
    }
}
