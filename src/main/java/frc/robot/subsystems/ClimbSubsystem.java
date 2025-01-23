// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final SparkMax climbMotor = new SparkMax(1, MotorType.kBrushless);
  private final RelativeEncoder climbMotorEncoder = climbMotor.getEncoder();

  private final PIDController controller = new PIDController(0.1, 0, 0);

  public ClimbSubsystem() {}

  public void setSpeed(double spud) {
    climbMotor.set(spud);
  }

  public void setPosition(double position) {
    double currentPosition = climbMotorEncoder.getPosition();
    double speed = controller.calculate(currentPosition, position);
    setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
