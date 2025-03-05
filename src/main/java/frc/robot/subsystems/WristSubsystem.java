// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.MotorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;

public class WristSubsystem extends SubsystemBase {
  private final Servo armServoMotor = new Servo(DeviceIds.kArmMotor);
  private final Servo wristServoMotor = new Servo(DeviceIds.kWristMotor);

  /** Creates a new WristSubsystem. */  
  public WristSubsystem() {}

  public void setArmMotorPosition(double desiredPosition) {
    armServoMotor.setAngle(desiredPosition);
  }

  public void setWristMotorPostition(double desiredPosition) {
    wristServoMotor.setAngle(desiredPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
