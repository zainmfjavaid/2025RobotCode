package frc.robot.hardware;

import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorController {
    public static final class MotorDefaults{
        //Constants to use as default values for Motor Controllers
        public static final int kCurrentLimit = 40;
        public static final double kOpenLoopRampRate = 0.2;
    }

    public static enum MotorConfig {
        // Swerve Modules
        FrontLeftModuleDrive(Constants.DeviceIds.kFrontLeftDriveMotor, 50, true),
        FrontLeftModuleTurn(Constants.DeviceIds.kFrontLeftAngleMotor, 40, true, true),
        FrontRightModuleDrive(Constants.DeviceIds.kFrontRightDriveMotor, 50, true),
        FrontRightModuleTurn(Constants.DeviceIds.kFrontRightAngleMotor, 40, true, true),
        BackLeftModuleDrive(Constants.DeviceIds.kBackLeftDriveMotor, 50, true),
        BackLeftModuleTurn(Constants.DeviceIds.kBackLeftAngleMotor, 40, true, true),
        BackRightModuleDrive(Constants.DeviceIds.kBackRightDriveMotor, 50, true),
        BackRightModuleTurn(Constants.DeviceIds.kBackRightAngleMotor, 40, true, true);

        private int ID;
        private int currentLimit;
        private boolean isBrake;
        private double openLoopRampRate;
        private boolean isReversed;

        MotorConfig(int ID, int currentLimit, boolean isBrake, double openLoopRampRate, boolean isReversed) {
            this.ID = ID;
            this.currentLimit = currentLimit;
            this.isBrake = isBrake;
            this.openLoopRampRate = openLoopRampRate;
            this.isReversed = isReversed;
        }

        MotorConfig(int ID, int currentLimit, boolean isBrake, double openLoopRampRate){
            this(ID, currentLimit, isBrake, openLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, boolean isBrake, boolean reversed){
            this(ID, currentLimit, isBrake, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit, boolean isBrake) {
            this(ID, currentLimit, isBrake, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit){
            this(ID, currentLimit, false, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, boolean reversed){
            this(ID, MotorDefaults.kCurrentLimit, false, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID){
            this(ID, MotorDefaults.kCurrentLimit, false, MotorDefaults.kOpenLoopRampRate, false);
        }

        public int getID(){
            return ID;
        }

        public int getCurrentLimit(){
            return currentLimit;
        }

        public boolean getIsBrake(){
            return isBrake;
        }

        public double getOpenLoopRampRate(){
            return openLoopRampRate;
        }

        public boolean getReversed(){
            return isReversed;
        }
    }

    public static KrakenMotor constructMotor(MotorConfig config) {
        return new KrakenMotor(config.getID(), config.getIsBrake(), config.getReversed(), config.getReversed());
    }

    // public static CANSparkMax constructMotor(MotorConfig config){
    //     CANSparkMax motor = new CANSparkMax(config.getID(), MotorType.kBrushless);
    //     motor.restoreFactoryDefaults();
    //     motor.setSmartCurrentLimit(config.getCurrentLimit());
    //     motor.setIdleMode(config.getIdleMode());
    //     motor.setOpenLoopRampRate(config.getOpenLoopRampRate());
    //     motor.setInverted(config.getReversed());
    //     return motor;
    // }

    // public static CANSparkMax constructMotor(MotorConfig config, double[] PIDArray){
    //     CANSparkMax motor = constructMotor(config);
    //     SparkPIDController motorPIDcontroller = motor.getPIDController();
    //     motorPIDcontroller.setP(PIDArray[0]);
    //     motorPIDcontroller.setI(PIDArray[1]);
    //     motorPIDcontroller.setD(PIDArray[2]);
    //     return motor;
    // }
}