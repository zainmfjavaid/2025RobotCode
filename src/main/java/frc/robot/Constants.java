// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.subsystems.SwerveUtils;

public final class Constants {
    public static final double kTau = Math.PI * 2;

    public static final double aprilTagHeightInches = 12.125;
    public static final double cameraHeightInches = 12.5;

    public static enum RobotMode {
        RUN, TEST;
    }

    public static final RobotMode kRobotMode = RobotMode.TEST;

    public static class RobotConstants {
        public static final double kTrackWidth = Units.inchesToMeters(19.75); // don't know if this is right
        public static final double kWheelbase = Units.inchesToMeters(19.75); // don't know if this is right

        public static final double kKrakenMotorMaxRotationsPerMinute = 6000;
        public static final double kKrakenMotorMaxRadiansPerSecond = kKrakenMotorMaxRotationsPerMinute / 60 * kTau;
    }

    public static class DriveConstants {
        public static enum DriveType {
            ARCADE, // doesn't exist
            SWERVE, 
            SPINDRIVE, // spins the drive motors // used to determine direction of drive motors
            SPINANGLE, // spins the angle motors // used to determine direction of angle motors
            SPIN, // set rotation speed
            DRIVE, // set longitudinal speed
            ALIGN, // doesn't do anything
            TEST;
        } 
        public static final DriveType kDriveType = DriveType.SPINDRIVE;
    }

    public static class SwerveConstants {
        public enum Module {
            FRONT_LEFT(DeviceIds.kFrontLeftDriveMotor, DeviceIds.kFrontLeftAngleMotor, new Translation2d(RobotConstants.kWheelbase / 2, RobotConstants.kTrackWidth / 2), EncoderConfig.FRONT_LEFT, "Front Left"),
            FRONT_RIGHT(DeviceIds.kFrontRightDriveMotor, DeviceIds.kFrontRightAngleMotor, new Translation2d(RobotConstants.kWheelbase / 2, -RobotConstants.kTrackWidth / 2), EncoderConfig.FRONT_RIGHT, "Front Right"),
            BACK_LEFT(DeviceIds.kBackLeftDriveMotor, DeviceIds.kBackLeftAngleMotor, new Translation2d(-RobotConstants.kWheelbase / 2, RobotConstants.kTrackWidth / 2), EncoderConfig.BACK_LEFT, "Back Left"),
            BACK_RIGHT(DeviceIds.kBackRightDriveMotor, DeviceIds.kBackRightAngleMotor, new Translation2d(-RobotConstants.kWheelbase / 2, -RobotConstants.kTrackWidth / 2), EncoderConfig.BACK_RIGHT, "Back Right");

            private final int driveMotorDeviceId;
            private final int angleMotorDeviceId;
            private final Translation2d location;
            private final EncoderConfig encoderConfig;
            private final String name;

            private Module(int driveMotorDeviceId, int angleMotorDeviceId, Translation2d location, EncoderConfig encoderConfig, String name) {
                this.driveMotorDeviceId = driveMotorDeviceId;
                this.angleMotorDeviceId = angleMotorDeviceId;
                this.location = location;
                this.encoderConfig = encoderConfig;
                this.name = name;
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
            public String getName() {
                return name;
            }
        }
        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5); // don't know if this is right
        public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;

        public static final double kDriveMotorGearRatio = 1.0 / 5.0; // I don't know if this is right
        public static final double kAngleMotorGearRatio = 1.0 / 3.0 / 4.0;

        public static final double kMaxWheelDriveSpeedRadiansPerSecond = SwerveUtils.driveMotorToWheel(RobotConstants.kKrakenMotorMaxRadiansPerSecond);
        public static final double kMaxWheelDriveSpeedMetersPerSecond = SwerveUtils.getLinearVelocity(kMaxWheelDriveSpeedRadiansPerSecond, kWheelRadiusMeters);

        public static final double kRobotRotationRadiusMeters = Math.hypot(RobotConstants.kTrackWidth / 2, RobotConstants.kWheelbase / 2);
        public static final double kMaxRotationSpeedRadiansPerSecond = kMaxWheelDriveSpeedMetersPerSecond / kRobotRotationRadiusMeters;

        // kP should be between 0 to 1
        public static final PIDController kAngleController = new PIDController(0.5, 0, 0);

        // YOU CAN CHANGE VALUES IN HERE
        public static class TeleopSwerveConstants {
            public static final double kMaxDriveSpeedMetersPerSecond = kRobotMode == RobotMode.TEST ? Units.feetToMeters(2) : kMaxWheelDriveSpeedMetersPerSecond * 0.75;
            
            public static final double kMaxRotationSpeedRadiansPerSecond = SwerveConstants.kMaxRotationSpeedRadiansPerSecond / 6;
        }

        public static class AutoSwerveConstants {
            public static final double kMaxDriveSpeedMetersPerSecond = Units.feetToMeters(2);
            public static final double kMaxRotationSpeedRadiansPerSecond = Math.PI / 6;

            public static final double kMaxAccelerationMetersPerSecondSquared = kMaxDriveSpeedMetersPerSecond / 6;
            public static final double kMaxRotationAccelerationRadiansPerSecondSquared = kMaxRotationSpeedRadiansPerSecond / 6;

            public static final PIDController kXController = new PIDController(0.1, 0, 0.1);
            public static final PIDController kYController = new PIDController(0.1, 0, 0.1);
            public static final PIDController kThetaController = new PIDController(0.1, 0, 0);
        }
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class AbsoluteEncoderConstants {
        public static final double kFrontLeftOffset = -(0.29296875);
        public static final double kFrontRightOffset = -(0.46142578125);
        public static final double kBackLeftOffset = -(0.3974609375);
        public static final double kBackRightOffset = -(-0.19677734375);

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
        public static final int kArmMotor = 32;
        public static final int kWristMotor = 33;
        public static final int kKickerMotor = 31;
        public static final int kRollerMotor = 30;
    }

    public static class IntakeConstants {
        public enum IntakeState {
            // ARM VALUES/WRIST ANGLES ARE TEMPORARY
            STOW(-0.6, 0, 0),
            INTAKE(-10, 0, 0),
            TROUGH(2, 0, -10500),
            L2(2, 0, -17000),
            L3(5, 0, -19000),
            L4(8, 0, -25000),
            TORCH(0, 0, 0);

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
    }

    public static class ElevatorConstants {
        public static final double elevatorSpeed = kRobotMode == RobotMode.TEST ? 0.4 : 1;
    }
}
