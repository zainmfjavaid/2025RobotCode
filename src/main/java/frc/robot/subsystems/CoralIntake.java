// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralIntake extends SubsystemBase {
  /** Creates a new CoralIntake. */
  SparkMax coralIntakeMoter = new SparkMax(0, MotorType.kBrushless);
  boolean inOut;
  public CoralIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setSpeed(double speed){
    if(inOut){
    coralIntakeMoter.set(speed);
    inOut = false;
    }
    else{
    coralIntakeMoter.set(-speed);
    inOut = true;  
    }
  }  
}
