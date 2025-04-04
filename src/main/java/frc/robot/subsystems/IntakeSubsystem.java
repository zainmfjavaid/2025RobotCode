// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.SystemSpeeds;

public class IntakeSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */
    SparkMaxMotor armMotor = new SparkMaxMotor(DeviceIds.kArmMotor, true, true, true);
    SparkMaxMotor wristMotor = new SparkMaxMotor(DeviceIds.kWristMotor, true, true, true);
    SparkMaxMotor rollerMotor = new SparkMaxMotor(DeviceIds.kRollerMotor);
    
    PIDController armPIDController = new PIDController(0.04, 0, 0);
    PIDController wristPIDController = new PIDController(0.04, 0, 0);

    IntakeState currentGoal = null;
    LaserCan lc = new LaserCan(22);

    public IntakeSubsystem() {}

    public double getDistance() {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm;
        } else {
            return 999.9;
        }
    }

    public boolean getHasMeasurement() {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null) {
            return measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
        } else {
            return false;
        }
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

    public StartEndCommand intakeCommand() {
        return new StartEndCommand(() -> runRollerMotors(0.6), () -> runRollerMotors(0), this);
    }

    public StartEndCommand runWristForward() {
        return new StartEndCommand(() -> wristMotor.set(0.1), () -> stopWristMotor(), this);
    }

    public StartEndCommand runWristBackward() {
        return new StartEndCommand(() -> wristMotor.set(-0.1), () -> stopWristMotor(), this);
    }

    public StartEndCommand runArmForward() {
        return new StartEndCommand(() -> armMotor.set(0.1), () -> armMotor.set(0), this);
    }

    public StartEndCommand runArmBackward() {
        return new StartEndCommand(() -> armMotor.set(-0.1), () -> armMotor.set(0), this);
    }

    public InstantCommand printEncoderValues() {
        return new InstantCommand(() -> {System.out.println("W: " + wristMotor.getPositionRotations() + "\nA: " + armMotor.getPositionRotations());}, this);
    }

    public boolean intakeHasCoral() {
        return getDistance() == 0.0 && getHasMeasurement();
    }

    @Override
    public void periodic() {
        //System.out.println("Lasercan reading:  " + getDistance());
        if (currentGoal != null) {
            setPosition(currentGoal);
        }
    }
}
