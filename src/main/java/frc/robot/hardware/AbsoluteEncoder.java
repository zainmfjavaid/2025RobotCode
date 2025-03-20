package frc.robot.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class AbsoluteEncoder {
    public enum EncoderConfig {
        //Swerve Modules (CAN)
        //Offsets determined by manually turning all modules to 0 (forward) and recording their positions
        FrontLeftModule(Constants.DeviceIds.kFrontLeftCancoder, false, -(0.30224609375)),
        FrontRightModule(Constants.DeviceIds.kFrontRightCancoder, false, -(0.462646484375)),
        BackLeftModule(Constants.DeviceIds.kBackLeftCancoder, false, -(0.41650390625)),
        BackRightModule(Constants.DeviceIds.kBackRightCancoder, false, -(-0.199951171875));

        private int ID;
        private boolean reversed;
        private double offset;
    
        EncoderConfig(int ID, boolean reversed, double offset){
            this.ID = ID;
            this.reversed = reversed;
            this.offset = offset;
        }
    
        EncoderConfig(int ID, boolean reversed){
            this(ID, reversed, 0);
        }
    
        EncoderConfig(int ID){
            this(ID, false, 0);
        }
    
        public int getID(){
            return ID;
        }

        public boolean getReversed(){
            return reversed;
        }

        public double getOffset(){
            return offset;
        }
    }

    public static CANcoder constructCANCoder(EncoderConfig config) {
        CANcoder encoder = new CANcoder(config.getID(), "rio");

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();

        magConfig.withAbsoluteSensorDiscontinuityPoint(0.5);
        if (config.getReversed()) 
            magConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        else
            magConfig.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        magConfig.withMagnetOffset(Units.radiansToRotations(config.getOffset()));

        canConfig.withMagnetSensor(magConfig);
        encoder.getConfigurator().apply(canConfig);

        return encoder;
    }

    public double getPositionRotations(CANcoder encoder) {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public static double getPositionRadians(CANcoder encoder) {
        return Units.rotationsToRadians(encoder.getAbsolutePosition().getValueAsDouble());
    }

    public static double getPositionRadians(CANcoder encoder, int places){
        //Truncates measure to places decimal points
        return Math.round(getPositionRadians(encoder) * Math.pow(10, places)) / Math.pow(10, places);
    }
}