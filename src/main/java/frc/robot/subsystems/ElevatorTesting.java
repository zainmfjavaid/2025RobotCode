// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;

public class ElevatorTesting extends SubsystemBase {
  /** Creates a new ElevatorTesting. */
  SparkMaxMotor motorOne = new SparkMaxMotor(5, false, false, true);
  SparkMaxMotor motorTwo = new SparkMaxMotor(6, true, false, true);

  public ElevatorTesting() {
  }

  public void stop() {
    motorOne.set(0);
    motorTwo.set(0);
  }

  public void goUp() {
    motorOne.set(1);
    motorTwo.set(1);
  }

  public void goDown() {
    motorOne.set(-0.15);
    motorTwo.set(-0.15);
  }

  public StartEndCommand goUpCommand() {
    return new StartEndCommand(() -> goUp(), () -> stop(), this);
  }

  public StartEndCommand goDownCommand() {
    return new StartEndCommand(() -> goDown(), () -> stop(), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}