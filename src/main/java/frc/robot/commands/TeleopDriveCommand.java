package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.SystemSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveModule;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    @SuppressWarnings("unused")
    private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
	@SuppressWarnings("unused")
    private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
	@SuppressWarnings("unused")
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
    private final DoubleSupplier translationX, translationY, rotationX, rotationY;
    
    private TalonFX[] angleMotors;
    private TalonFX[] driveMotors;
    
    // PID constants for angle control
    private final double kP = 0.01;
    private final double maxOutput = 0.5;

    boolean fieldCentricRotation = false;
    double maxOmegaScale = 0.3;

    PIDController omegaController = new PIDController(0.5, 0, 0);
    
    public TeleopDriveCommand(SwerveSubsystem swerve, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotationX, DoubleSupplier rotationY) {
        this.swerve = swerve;
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotationX = rotationX;
        this.rotationY = rotationY;

        omegaController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
        
        addRequirements(swerve);
    }
    
    private TalonFX talonFXinverted(TalonFX talon, Boolean isInverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        talon.setNeutralMode(NeutralModeValue.Brake);
        motorOutput.Inverted = isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        config.MotorOutput = motorOutput;
        talon.getConfigurator().apply(config);
        return talon;
    }
    
    @Override
    public void initialize() {
        // Initialize motors
        angleMotors = new TalonFX[4];
        driveMotors = new TalonFX[4];

        // Front Left
        angleMotors[0] = new TalonFX(DeviceIds.kFrontLeftAngleMotor, "CANivore2158");
        driveMotors[0] = new TalonFX(DeviceIds.kFrontLeftDriveMotor, "CANivore2158");        
        angleMotors[0] = talonFXinverted(angleMotors[0], false);
        driveMotors[0] = talonFXinverted(driveMotors[0], false);

        // Front Right
        angleMotors[1] = new TalonFX(DeviceIds.kFrontRightAngleMotor, "CANivore2158");
        driveMotors[1] = new TalonFX(DeviceIds.kFrontRightDriveMotor, "CANivore2158");
        angleMotors[1] = talonFXinverted(angleMotors[1], false);
        driveMotors[1] = talonFXinverted(driveMotors[1], true);

        // Back Left
        angleMotors[2] = new TalonFX(DeviceIds.kBackLeftAngleMotor, "CANivore2158");
        driveMotors[2] = new TalonFX(DeviceIds.kBackLeftDriveMotor, "CANivore2158");
        angleMotors[2] = talonFXinverted(angleMotors[2], false);
        driveMotors[2] = talonFXinverted(driveMotors[2], false);

        // Back Right
        angleMotors[3] = new TalonFX(DeviceIds.kBackRightAngleMotor, "CANivore2158");
        driveMotors[3] = new TalonFX(DeviceIds.kBackRightDriveMotor, "CANivore2158");
        driveMotors[3] = talonFXinverted(driveMotors[3], true);
        angleMotors[3] = talonFXinverted(angleMotors[3], false);

        for (int i = 0; i < 4; i++) {
            driveMotors[i].getConfigurator().apply(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.kDriveCurrentLimit));
            angleMotors[i].getConfigurator().apply(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Constants.kAngleCurrentLimit));
        }
    }
    
    @Override
    public void execute() {
        // Get joystick inputs with deadband
        double xInput = Math.pow(applyDeadband(translationX.getAsDouble()), 3);
        double yInput = Math.pow(applyDeadband(translationY.getAsDouble()), 3);
        
        double xOmegaInput = Math.pow(applyDeadband(rotationX.getAsDouble()), 3); 
        double yOmegaInput = Math.pow(applyDeadband(rotationY.getAsDouble()), 3);
        double desiredOmegaRad = Math.atan2(xOmegaInput, yOmegaInput);

        // Scale inputs to correct speeds
        double xSpeed = xInput * SystemSpeeds.kMaxDriveSpeedMetersPerSecond;
        double ySpeed = yInput * SystemSpeeds.kMaxDriveSpeedMetersPerSecond;

        double omegaRadPerSec;

        if (fieldCentricRotation) {
            // This allows the robot to stay in the last rotation
            if (Math.hypot(xOmegaInput, yOmegaInput) > 0.1) {

                double omegaOutput = omegaController.calculate(
                    swerve.getGyroAngle().getRadians(), desiredOmegaRad);

                omegaRadPerSec = omegaOutput / (Math.PI * 2) * 
                    SystemSpeeds.kMaxRotationSpeedRadiansPerSecond * maxOmegaScale;
            } else {
                omegaRadPerSec = 0;
            }
        } else {
            omegaRadPerSec = xOmegaInput * SystemSpeeds.kMaxRotationSpeedRadiansPerSecond;
        }

        if (Constants.isDemo) {
            xSpeed = xInput/4;
            ySpeed = yInput/4;
        }
        
        // Create chassis speeds (field or robot relative)
        ChassisSpeeds speeds;
        if (SwerveSubsystem.isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaRadPerSec, swerve.getGyroAngle());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, omegaRadPerSec);
        }
        
        // Get module states using YAGSL's kinematics
        SwerveModuleState[] moduleStates = swerve.getKinematics().toSwerveModuleStates(speeds);
        
        // Optional: Desaturate module states to respect max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SystemSpeeds.kMaxDriveSpeedMetersPerSecond);
        
        // Control each module
        SwerveModule[] modules = swerve.getSwerveDrive().getModules();
        for (int i = 0; i < moduleStates.length; i++) {
            // Get current angle
            double currentAngle = modules[i].getState().angle.getDegrees();
            
            // Optimize state to avoid unnecessary rotation
            // Create a new SwerveModuleState object and optimize it
            SwerveModuleState optimizedState = moduleStates[i];
            optimizedState.optimize(Rotation2d.fromDegrees(currentAngle));

            //moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(currentAngle));
            
            // Angle control with continuous wrapping
            double targetAngle = moduleStates[i].angle.getDegrees();
            double angleError = targetAngle - currentAngle;
            
            // Wrap error to -180 to 180
            angleError = angleError % 360;
            if (angleError > 180) angleError -= 360;
            if (angleError < -180) angleError += 360;
            
            // Apply simple P control for angle with limiting
            double angleOutput = angleError * kP;
            // TODO: Dont divide by 4
            angleOutput = Math.max(-maxOutput/4, Math.min(maxOutput, angleOutput));
            
            // Calculate drive output normalized to [-1, 1]
            double driveOutput = moduleStates[i].speedMetersPerSecond / SystemSpeeds.kMaxDriveSpeedMetersPerSecond;
            
            // Send commands to motors
            angleMotors[i].set(angleOutput);
            driveMotors[i].set(driveOutput);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop all motors when command ends
        for (int i = 0; i < 4; i++) {
            if (angleMotors[i] != null) angleMotors[i].set(0);
            if (driveMotors[i] != null) driveMotors[i].set(0);
        }
    }
    
    // Helper method for deadband
    private double applyDeadband(double value) {
        final double deadband = 0.1;
        if (Math.abs(value) < deadband) {
            return 0;
        }
        // Scale remaining values to full range
        return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
    }
}