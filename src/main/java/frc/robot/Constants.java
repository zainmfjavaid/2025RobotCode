// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

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

  public static class DriveConstants {
    public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(16);
  }

  public static class SwerveConstants {
    public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
  } 

  public static class TeleopSwerveConstants {
    public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2);
    public static final double kMaxRotationSpeedRadiansPerSecond = Math.PI / 6;

    public static final PIDController kRotationController = new PIDController(0.1 / Math.PI, 0, 0);
  }

  public static class AutoSwerveConstants {
    // these values are temporary
    public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2);
    public static final double kMaxRotationSpeedRadiansPerSecond = Math.PI / 6;

    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxDriveSpeedMetersPerSecond / 6;
    public static final double kMaxRotationAccelerationRadiansPerSecondSquared = kMaxRotationSpeedRadiansPerSecond / 6;

    public static final PIDController kXController = new PIDController(16, 0, 0.1);
    public static final PIDController kYController = new PIDController(16, 0, 0.1);
    public static final PIDController kThetaController = new PIDController(0.5 / Math.PI, 0, 0);

    public static final Constraints kThetaConstraints = new Constraints(kMaxRotationSpeedRadiansPerSecond, kMaxRotationAccelerationRadiansPerSecondSquared);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AbsoluteEncoderConstants {
    /*
     * HOW TO GET THE VALUES (this might not work)
     * set the offsets to 0
     * run the robot on arcade drive, and align the wheels so that they are all facing forward
     * record the absolute encoder values
     * set the offsets to the negative of the recorded values
     * (To see the offsets, you have to run twice for some reason.)
     */

    // possibly fix these
    public static final double kFrontLeftOffset = -1.7441 / Constants.kTau;
    public static final double kFrontRightOffset = 2.0678 / Constants.kTau;
    public static final double kBackLeftOffset = (-2.0801 + Math.PI) / Constants.kTau;
    public static final double kBackRightOffset = 2.8041 / Constants.kTau;

    public static final double kAbsoluteSensorDiscontinuityPoint = 0.5;
  }

  public static class MotorConstants {
    public static final int kArmServoChannel = 15;
    public static final int kWristServoChannel = 16;
  }

  public static class IntakeConstants {
    public enum IntakeState {
      GroundIntake(0, 0), 
      SourceIntake(45, 0), 
      CoralScore(45, 90), 
      Stow(90, 90);
  
      public final double armAngle;
      public final double wristAngle;
  
      private IntakeState(double armAngle, double wristAngle) {
        this.armAngle = armAngle;
        this.wristAngle = wristAngle;
      }
    } 
  }
}
