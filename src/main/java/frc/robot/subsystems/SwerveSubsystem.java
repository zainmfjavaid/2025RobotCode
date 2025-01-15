// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;

public class SwerveSubsystem extends SubsystemBase {
  private final SparkMax frontLeftDrive = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax frontRightDrive = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax backLeftDrive = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax backRightDrive = new SparkMax(7, MotorType.kBrushless);

  private final SparkMax frontLeftAngle = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax frontRightAngle = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax backLeftAngle = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax backRightAngle = new SparkMax(7, MotorType.kBrushless);

  private final RelativeEncoder frontLeftEncoder = frontLeftAngle.getEncoder();
  private final RelativeEncoder frontRightEncoder = frontRightAngle.getEncoder();
  private final RelativeEncoder backLeftEncoder = backLeftAngle.getEncoder();
  private final RelativeEncoder backRightEncoder = backLeftAngle.getEncoder();

  private final PIDController wheelPID = new PIDController(0.1, 0, 0);

  private static final double width = Units.inchesToMeters(19.75);
  private static final double length = Units.inchesToMeters(19.75);

  private static final Translation2d frontLeftLocation = new Translation2d(width/2, length/2);
  private static final Translation2d frontRightLocation = new Translation2d(width/2, -length/2);
  private static final Translation2d backLeftLocation = new Translation2d(-width/2, length/2);
  private static final Translation2d backRightLocation = new Translation2d(-width/2, -length/2);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final double maxSpeedMetersPerSecond = 1;

  /** Creates a new TankDriveSubsystem. */
  public SwerveSubsystem() {}

  public void swerveDrive(double xSpeed, double ySpeed, double turnSpeed) {
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);

    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    
    frontLeftDrive.set(moduleStates[0].speedMetersPerSecond / maxSpeedMetersPerSecond);
    frontRightDrive.set(moduleStates[1].speedMetersPerSecond / maxSpeedMetersPerSecond);
    backLeftDrive.set(moduleStates[2].speedMetersPerSecond / maxSpeedMetersPerSecond);
    backRightDrive.set(moduleStates[3].speedMetersPerSecond / maxSpeedMetersPerSecond);

    setAngleMotorSpeed(frontLeftAngle, frontLeftEncoder, moduleStates[0].angle.getDegrees());
    setAngleMotorSpeed(frontRightAngle, frontRightEncoder, moduleStates[1].angle.getDegrees());
    setAngleMotorSpeed(backLeftAngle, backLeftEncoder, moduleStates[2].angle.getDegrees());
    setAngleMotorSpeed(backRightAngle, backRightEncoder, moduleStates[3].angle.getDegrees());
  }

  public void setAngleMotorSpeed(SparkMax motor, RelativeEncoder encoder, double angleDegrees) {
    double currentPosition = encoder.getPosition();
    double desiredPosition = angleDegrees;
    double speed = wheelPID.calculate(currentPosition, desiredPosition);
    motor.set(speed);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
