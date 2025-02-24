// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.DriveUtils;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kTau = Math.PI * 2;

  public static final double kPeriodicDuration = 0.03; // 30 milliseconds // unused

  public static class RobotConstants {
    public static final double kWidthMeters = Units.inchesToMeters(19.75);
    public static final double kLengthMeters = Units.inchesToMeters(19.75);

    public static final double kKrakenMotorMaxRotationsPerMinute = 6000;
    public static final double kKrakenMotorMaxRadiansPerSecond = kKrakenMotorMaxRotationsPerMinute / 60 * kTau;
  }
  
  public static class DriveConstants {
    public static enum DriveType {
      ARCADE, 
      SWERVE, 
      SPINDRIVE, // spins the drive motors // used to determine direction of drive motors
      SPINANGLE, // spins the angle motors // used to determine direction of angle motors
      SPIN, // set rotation speed
      DRIVE, // set longitudinal speed
      ALIGN,
      TEST;
    }
    public static final DriveType driveType = DriveType.SWERVE;
    
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;

    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);

    private static final double kMaxWheelDriveSpeedRadiansPerSecond = DriveUtils.driveMotorToWheel(RobotConstants.kKrakenMotorMaxRadiansPerSecond);
    
    // ignore this - it doesn't work
    public static final double kMaxWheelAngleSpeedRadiansPerSecond = DriveUtils.angleMotorToWheel(RobotConstants.kKrakenMotorMaxRadiansPerSecond);
    //


    public static final double kMaxWheelDriveSpeedMetersPerSecond = kMaxWheelDriveSpeedRadiansPerSecond * kWheelRadiusMeters;

    private static final double kRotationRadiusMeters = Math.hypot(RobotConstants.kWidthMeters / 2, RobotConstants.kLengthMeters / 2);
    public static final double kMaxRotationSpeedRadiansPerSecond = kMaxWheelDriveSpeedMetersPerSecond / kRotationRadiusMeters;
  }

  public static class TeleopSwerveConstants {
    // if you want to change speeds

    public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2);

    public static final double kMaxRotationSpeedRadiansPerSecond = DriveConstants.kMaxRotationSpeedRadiansPerSecond / 6;

    public static final PIDController kRotationController = new PIDController(0.85, 0, 0);
  }

  public static class AutoSwerveConstants {
    public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2);
    public static final double kMaxRotationSpeedRadiansPerSecond = Math.PI / 6;

    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxDriveSpeedMetersPerSecond / 6;
    public static final double kMaxRotationAccelerationRadiansPerSecondSquared = kMaxRotationSpeedRadiansPerSecond / 6;

    public static final PIDController kXController = new PIDController(0.1, 0, 0.1);
    public static final PIDController kYController = new PIDController(0.1, 0, 0.1);

    public static final PIDController kThetaController = new PIDController(0.5 / Math.PI, 0, 0);
    public static final Constraints kThetaConstraints = new Constraints(kMaxRotationSpeedRadiansPerSecond, kMaxRotationAccelerationRadiansPerSecondSquared);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class AbsoluteEncoderConstants {
  /*
  FL: RRot -0.0, RRad -0.0, ARot 0.288330078125, ARad 1.8116313104929422
FR: RRot -0.0, RRad -0.0, ARot 0.4794921875, ARad 3.0127382674073995
BL: RRot -0.0, RRad -0.0, ARot 0.396728515625, ARad 2.492718780314167
BR: RRot -0.0, RRad -0.0, ARot -0.18994140625, ARad -1.193437052975029

  */

    public static final double kFrontLeftOffset = -(.288330078125);
    public static final double kFrontRightOffset = -(.4794921875); // -(-0.19140625);
    public static final double kBackLeftOffset = -(.396728515625); // -(0.15185546875);
    public static final double kBackRightOffset = -(-.18994140625); // -(0.208984375);

    public static final double kAbsoluteSensorDiscontinuityPoint = 0.5;
  }

  public static class MotorConstants {
    public static final int kFrontLeftDriveMotorDeviceId = 1;
    public static final int kFrontLeftAngleMotorDeviceId = 2;
    public static final int kFrontRightDriveMotorDeviceId = 8;
    public static final int kFrontRightAngleMotorDeviceId = 7;
    public static final int kBackLeftDriveMotorDeviceId = 3;
    public static final int kBackLeftAngleMotorDeviceId = 4;
    public static final int kBackRightDriveMotorDeviceId = 5;
    public static final int kBackRightAngleMotorDeviceId = 6;

    public static final int kAlgaeRollerMotorID = 17;
    public static final int kAlgaeServoDeviceID = 1;
    public static final int kArmMotorId = 15;
    public static final int kWristServoChannel = 16;
    public static final int kIntakeMoterID = 18;
  }

  public static class IntakeConstants {
    public static final double kDeployPosition = 12;
    public static final double kRetractPosition = 0;
    public static final double kP = 0.04;
  }

  //Algae Constants
  public static enum IntakePosition {
    Retract(0),
    Deploy(60);

    private int intakePosition;
    
    IntakePosition(int intakePosition) {
      this.intakePosition = intakePosition;
    }

    public int getPosition() {
      return intakePosition;
    }

  }

  public static final double algaeIntakeRollerSpeed = 1;
  public static final double algaeOuttakeRollerSpeed = -1;
}
