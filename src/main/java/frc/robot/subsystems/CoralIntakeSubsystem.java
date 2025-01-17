// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() {}

  
private final SparkMax intakeMotor = new SparkMax(7, MotorType.kBrushless);


public void setIntakeMotor(double speed) {
  intakeMotor.set(speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
