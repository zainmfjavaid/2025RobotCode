// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.MotorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WristSubsystem extends SubsystemBase {

  public enum IntakeState {
    GroundIntake(0, 0), 
    SourceIntake(45, 0), 
    CoralScore(45, 90), 
    Stow(90, 90);

    public final double armAngle;
    public final double wristAngle;

    private IntakeState(double armAngle, double wristAngle) {
      this.armAngle = armAngle;
      this.wristAngle = wristAngle;
    }
  } 

  private final SparkMax armMotor = new SparkMax(MotorConstants.kWristMotorId, MotorType.kBrushless);
  private final SparkMax intakeMotor = new SparkMax(MotorConstants.kIntakeMoterId, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  
  private final Servo servoWristMotor = new Servo(MotorConstants.kWristServoChannel);

  private final PIDController wristPID = new PIDController(.1, .1,0);
  
  /** Creates a new WristSubsystem. */  
  public WristSubsystem() {}


  public void setArmMotorSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.set(0);
  }


  public void setArmMotorPosition(double desiredPosition) {
    double currentPosition = armEncoder.getPosition();
    double speed = wristPID.calculate(currentPosition, desiredPosition);
    setArmMotorSpeed(speed);
  }

  public void setWristMotorPostition(double desiredPosition) {
    servoWristMotor.setAngle(desiredPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
