// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    // Swerve Constants
    public static final double kTau = Math.PI * 2;
    public static final double kDriveMotorGearRatio = 1.0 / 5.0;
    public static final double kAngleMotorGearRatio = 1.0 / 3.0;

    // Vision Constants
    public static final double aprilTagHeightInches = 12.125;
    public static final double cameraHeightInches = 12.5;

    // Controller Constants
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Speed Constants
    public static class SystemSpeeds {
        public static final double kElevatorDownSpeed = 0.4;

        public static final double kIntakeRollerSpeed = 0.8;
        public static final double kOuttakeRollerSpeed = -0.6;
        public static final double kScoreOuttakeRollerSpeed = -0.2;

        public static final double kClimbSpeed = 1.0;
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

        // Climb (SparkMax)
        public static final int kClimbMotor = 35;
    }

    public static class IntakeConstants {
        public enum IntakeState {
            STOW(1.1, 16.9, 0),
            INTAKE(21.2, 16.9, 0),
            SOURCE(2.2, 16.9, -6200),
            TROUGH(6, 16.9, 0),
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
    }

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
