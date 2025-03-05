// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.SparkMaxMotor;
import edu.wpi.first.math.controller.PIDController;

public class ClimbSubsystem extends SubsystemBase {

  SparkMaxMotor climbMotor = new SparkMaxMotor(35); // TEMP

  PIDController climbPIDController = new PIDController(0.1, 0, 0);

  boolean climbToggle = false;

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {
    }

    public void setClimbDown() {
      climbMotor.set(climbPIDController.calculate(climbMotor.getPositionRotations(), 1.0));
    }

    public void setClimbUp() {
      climbMotor.set(climbPIDController.calculate(climbMotor.getPositionRotations(), 1.0)); // TEMP POSITION
    }

    public void setClimb() {
      if(climbToggle) setClimbDown(); else setClimbUp();
      climbToggle = climbToggle ? true : false;
    }

    public StartEndCommand climbCommandTest() {
      return new StartEndCommand(() -> climbMotor.set(1), () -> climbMotor.set(0), this);
    }

    public StartEndCommand reverseClimbCommandTest() {
      return new StartEndCommand(() -> climbMotor.set(-1), () -> climbMotor.set(0), this);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
