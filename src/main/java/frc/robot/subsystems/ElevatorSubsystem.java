// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  public enum ElevatorPosition {
    Ground(0),
    L1(1),
    L2(2),
    L3(3),
    L4(4);

    private double position;

    private ElevatorPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  private final SparkMax leftElevatorMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax rightElevatorMotor = new SparkMax(2, MotorType.kBrushless);

  private final RelativeEncoder leftMotorEncoder = leftElevatorMotor.getEncoder();
  private final RelativeEncoder rightMotorEncoder = rightElevatorMotor.getEncoder();

  private final PIDController controller = new PIDController(0.1, 0, 0);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  public void stop() {
    leftElevatorMotor.set(0);
    rightElevatorMotor.set(0);
  }

  public void setPosition(double position) {
    leftElevatorMotor.set(controller.calculate(leftMotorEncoder.getPosition(), position));
    rightElevatorMotor.set(controller.calculate(rightMotorEncoder.getPosition(), position));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
