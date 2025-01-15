// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {}

  private final SparkMax armMotor = new SparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  
  private final SparkMax servoWristMotor = new SparkMax(3, MotorType.kBrushless);
  private final RelativeEncoder servoWristEncoder = servoWristMotor.getEncoder();

  private final PIDController armPID = new PIDController(0, 0, 0);
  private final PIDController servoWristPID = new PIDController(0.1, 0, 0);



  public void setArmMotorSpeed(double speed) {
    armMotor.set(speed);
  }

  public void setServoWristMotorSpeed (double speed) {
    servoWristMotor.set(speed);
  }

  public void setArmMotorPosition(double desiredPosition) {
    double currentPosition = armEncoder.getPosition();
    double speed = armPID.calculate(currentPosition, desiredPosition);
    setArmMotorSpeed(speed);
  }

  public void setWristMotorPostition(double desiredPosition) {
    double currentPosition = servoWristEncoder.getPosition();
    double speed = servoWristPID.calculate(currentPosition, desiredPosition);
    setServoWristMotorSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
