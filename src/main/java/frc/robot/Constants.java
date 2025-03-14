// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.subsystems.DriveUtils;

public final class Constants {
    public static final double kTau = Math.PI * 2;

    public static final double aprilTagHeightInches = 12.125;
    public static final double cameraHeightInches = 12.5;

    public static class RobotConstants {
        public static final double kTrackWidth = Units.inchesToMeters(19.75); // don't know if this is right
        public static final double kWheelbase = Units.inchesToMeters(19.75); // don't know if this is right

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
        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5); // don't know if this is right
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;

        public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double kAngleMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);

        private static final double kMaxWheelDriveSpeedRadiansPerSecond = DriveUtils.driveMotorToWheel(RobotConstants.kKrakenMotorMaxRadiansPerSecond);
        public static final double kMaxWheelDriveSpeedMetersPerSecond = DriveUtils.getLinearVelocity(kMaxWheelDriveSpeedRadiansPerSecond, kWheelRadiusMeters);

        public static final double kMaxWheelAngleSpeedRadiansPerSecond = DriveUtils.angleMotorToWheel(RobotConstants.kKrakenMotorMaxRadiansPerSecond);

        private static final double kRotationRadiusMeters = Math.hypot(RobotConstants.kTrackWidth / 2, RobotConstants.kWheelbase / 2);
        public static final double kMaxRotationSpeedRadiansPerSecond = kMaxWheelDriveSpeedMetersPerSecond / kRotationRadiusMeters;
    }

    public static class SwerveConstants {
        public enum Module {
            FRONT_LEFT(DeviceIds.kFrontLeftDriveMotor, DeviceIds.kFrontLeftAngleMotor, new Translation2d(RobotConstants.kWheelbase / 2, RobotConstants.kTrackWidth / 2), EncoderConfig.FRONT_LEFT),
            FRONT_RIGHT(DeviceIds.kFrontRightDriveMotor, DeviceIds.kFrontRightAngleMotor, new Translation2d(RobotConstants.kWheelbase / 2, -RobotConstants.kTrackWidth / 2), EncoderConfig.FRONT_RIGHT),
            BACK_LEFT(DeviceIds.kBackLeftDriveMotor, DeviceIds.kBackLeftAngleMotor, new Translation2d(-RobotConstants.kWheelbase / 2, RobotConstants.kTrackWidth / 2), EncoderConfig.BACK_LEFT),
            BACK_RIGHT(DeviceIds.kBackRightDriveMotor, DeviceIds.kBackRightAngleMotor, new Translation2d(-RobotConstants.kWheelbase / 2, -RobotConstants.kTrackWidth / 2), EncoderConfig.BACK_RIGHT);

            private final int driveMotorDeviceId;
            private final int angleMotorDeviceId;
            private final Translation2d location;
            private final EncoderConfig encoderConfig;

            private Module(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig encoderConfig) {
                this.driveMotorDeviceId = driveMotorDeviceId;
                this.angleMotorDeviceId = angleMotorDeviceId;
                this.location = location;
                this.encoderConfig = encoderConfig;
            }

            public int getDriveMotorDeviceId() {
                return driveMotorDeviceId;
            }
            public int getAngleMotorDeviceId() {
                return angleMotorDeviceId;
            }
            public Translation2d getLocation() {
                return location;
            }
            public EncoderConfig getEncoderConfig() {
                return encoderConfig;
            }
        }
    }

    public static class TeleopSwerveConstants {
        public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2.5);

        public static final double kMaxRotationSpeedRadiansPerSecond = DriveConstants.kMaxRotationSpeedRadiansPerSecond / 6;

        public static final PIDController kRotationController = new PIDController(1.5, 0, 0);
    }

    public static class AutoSwerveConstants {
        public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(8);
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
        // if swerve module reverse is (true, true)
        // public static final double kFrontLeftOffset = -(0.29296875);
        // public static final double kFrontRightOffset = -(0.46142578125);
        // public static final double kBackLeftOffset = -(0.3974609375);
        // public static final double kBackRightOffset = -(-0.19677734375);

        /*
FL: RRot -0.2338346354166667, RRad -1.4692263455596954, ARot 0.30224609375, ARad 1.8990682154024239
FR: RRot -0.068359375, RRad -0.4295146206079795, ARot 0.462646484375, ARad 2.90689359304329
BL: RRot -0.698359375, RRad -4.3879213641311186, ARot 0.41650390625, ARad 2.6169712241329037
BR: RRot 0.456162109375, RRad 2.8661510633170475, ARot -0.199951171875, ARad -1.2563302652783401
         */
        
        // if swerve module reverse is (false, false)
        public static final double kFrontLeftOffset = -(0.30224609375);
        public static final double kFrontRightOffset = -(0.462646484375);
        public static final double kBackLeftOffset = -(0.41650390625);
        public static final double kBackRightOffset = -(-0.199951171875);

        public static final double kAbsoluteSensorDiscontinuityPoint = 0.5;
    }

    public static class DeviceIds {
        // Swerve (TalonFX)
        public static final int kFrontLeftDriveMotor = 1;
        public static final int kFrontLeftAngleMotor = 2;
        public static final int kFrontRightDriveMotor = 8;
        public static final int kFrontRightAngleMotor = 7;
        public static final int kBackLeftDriveMotor = 3;
        public static final int kBackLeftAngleMotor = 4;
        public static final int kBackRightDriveMotor = 5;
        public static final int kBackRightAngleMotor = 6;
        // Elevator (SparkMax)
        public static final int kLeftElevatorMotor = 5;
        public static final int kRightElevatorMotor = 6;
        // Intake (SparkMax)
        public static final int kArmMotor = 33;
        public static final int kWristMotor = 14;
        public static final int kRollerMotor = 30;
    }

    public static class IntakeConstants {
        public enum IntakeState {
            // ARM VALUES/WRIST ANGLES ARE TEMPORARY
            STOW(1.1, 77, 0),
            INTAKE(21.2, 77, 0),
            TROUGH(6, 77, 0),
            L2(12.8, 0, -6900),
            L3(12.8, 0, -14837),
            L4(12.8, 0, -31109),
            TORCH(17, 0, 0);

            private final double armValue;
            private final double wristValue;
            private final double elevatorValue;

            IntakeState(double armValue, double wristValue, double elevatorValue) {
                this.armValue = armValue;
                this.wristValue = wristValue;
                this.elevatorValue = elevatorValue;
            }

            public double getArmPosition() {
                return armValue;
            }

            public double getWristValue() {
                return wristValue;
            }

            public double getElevatorValue() {
                return elevatorValue;
            }
        }

        public static final double kickerWheelSpeed = 0.2;
        public static final double rollerMotorSpeed = 0.2;

        public static final double[] apriltagAngles = {
            0, // 0 (unused)
            0, // 1
            0, // 2
            0, // 3
            0, // 4
            0, // 5
            60, // 6
            0, // 7
            -60, // 8
            -120, // 9
            180, // 10
            120, // 11
            0, // 12
            0, // 13
            0, // 14
            0, // 15
            0, // 16
            60, // 17
            0, // 18
            -60, // 19
            -120, // 20
            180, // 21
            120 // 22
        };
    }
}
