package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax shooterAngleMotor = new SparkMax(17, MotorType.kBrushless);

    public void runShooterAngleMotor(double speed) {
        shooterAngleMotor.set(speed);
    }
}
