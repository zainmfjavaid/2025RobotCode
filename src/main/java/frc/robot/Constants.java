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
    public static final double kMaxWheelAngleSpeedRadiansPerSecond = DriveUtils.angleMotorToWheel(RobotConstants.kKrakenMotorMaxRadiansPerSecond);

    public static final double kMaxWheelDriveSpeedMetersPerSecond = kMaxWheelDriveSpeedRadiansPerSecond * kWheelRadiusMeters;

    private static final double kRotationRadiusMeters = Math.hypot(RobotConstants.kWidthMeters / 2, RobotConstants.kLengthMeters / 2);
    public static final double kMaxRotationSpeedRadiansPerSecond = kMaxWheelDriveSpeedMetersPerSecond / kRotationRadiusMeters;
  }

  public static class TeleopSwerveConstants {
    public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2);

    public static final double kMaxRotationSpeedRadiansPerSecond = DriveConstants.kMaxRotationSpeedRadiansPerSecond / 6;

    public static final PIDController kRotationController = new PIDController(0.8, 0, 0);
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
 * FL: RRot -1.73779296875, RRad -0.08109700520833334, ARot 0.0, ARad 0.0
FR: RRot 11.640380859375, RRad 0.5432177734375, ARot -0.19140625, ARad -4.1015625
BL: RRot 1.230224609375, RRad 0.057410481770833334, ARot 0.15185546875, ARad 3.254045758928571
BR: RRot -17.4873046875, RRad -0.8160742187500001, ARot 0.208984375, ARad 4.478236607142857

 */

    public static final double kFrontLeftOffset = 0;
    public static final double kFrontRightOffset = -0.19140625;
    public static final double kBackLeftOffset = 0.15185546875;
    public static final double kBackRightOffset = 0.208984375;

    public static final double kAbsoluteSensorDiscontinuityPoint = 0.5;
  }

  public static class MotorConstants {
    public static final int kIntakeDeployMotorDeviceId = 16;
    public static final int kIntakeRollerMotorDeviceId = 9;
    public static final int kIntakeIndexMotorDeviceId = 15; 

    public static final int kFrontLeftDriveMotorDeviceId = 1;
    public static final int kFrontLeftAngleMotorDeviceId = 2;
    public static final int kFrontRightDriveMotorDeviceId = 3;
    public static final int kFrontRightAngleMotorDeviceId = 4;
    public static final int kBackLeftDriveMotorDeviceId = 5;
    public static final int kBackLeftAngleMotorDeviceId = 6;
    public static final int kBackRightDriveMotorDeviceId = 8;
    public static final int kBackRightAngleMotorDeviceId = 7;
  }

  public static class IntakeConstants {
    public static final double kDeployPosition = 12;
    public static final double kRetractPosition = 0;
    public static final double kP = 0.04;
  }
}
