// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakePosition;



public class AlgaeSubsystem extends SubsystemBase {

    private SparkMax algaeRollerMotor;
    private Servo algaeServo;
    
  /** Creates a new AlgaeIntakeSubsystem. */
  public AlgaeSubsystem() {
   algaeRollerMotor = new SparkMax(0, MotorType.kBrushless);
   algaeServo = new Servo(1);
     }
   
  
  public void setRollerSpeed(double speed) {
    algaeRollerMotor.set(speed);
  }

  public void runRollersIntake() {
    setRollerSpeed(Constants.algaeIntakeRollerSpeed);
  }

  public void runRollersOuttake() {
    setRollerSpeed(Constants.algaeOuttakeRollerSpeed);
  }

  public void stopRollers() {
    setRollerSpeed(0);
  }

  public void setAlgaeDeployPosition(IntakePosition intakePosition) {
    algaeServo.setAngle(intakePosition.getPosition());
  }

  public void deployAlgae() { 
    setAlgaeDeployPosition(IntakePosition.Deploy);
  }  

  public void retractAlgae() {
    setAlgaeDeployPosition(IntakePosition.Retract);
  }
  ///public
  ///@Override
  ///public void periodic() {
    // This method will be called once per scheduler run
  ///}
}
