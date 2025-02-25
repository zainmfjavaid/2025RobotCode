// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.MotorConstants;
public class CoralIntakeSubsystem extends SubsystemBase {
  /** Creates a new CoralIntake. */
  private final SparkMax coralIntakeMoter = new SparkMax(MotorConstants.kIntakeMoterID, MotorType.kBrushless);

  public CoralIntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stopIntake(){
    setSpeed(0);
  }  

  public void setSpeed(double speed){
    coralIntakeMoter.set(speed);
  }  
}
