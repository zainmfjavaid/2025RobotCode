package frc.robot.commands;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.DeviceIds;
import frc.robot.Constants.SystemSpeeds;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveModule;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier translationX, translationY, rotation;
    private final boolean fieldRelative;
    
    private TalonFX[] angleMotors;
    private TalonFX[] driveMotors;
    
    // PID constants for angle control
    private final double kP = 0.01;
    private final double maxOutput = 0.5;
    
    public TeleopDriveCommand(SwerveSubsystem swerve, 
                              DoubleSupplier translationX, 
                              DoubleSupplier translationY, 
                              DoubleSupplier rotation,
                              boolean fieldRelative) {
        this.swerve = swerve;
        this.translationX = translationX;
        this.translationY = translationY;
        this.rotation = rotation;
        this.fieldRelative = fieldRelative;

        angleMotors = new TalonFX[4];
        driveMotors = new TalonFX[4];
        
        // Front Left
        angleMotors[0] = new TalonFX(DeviceIds.kFrontLeftAngleMotor, "CANivore2158");
        driveMotors[0] = new TalonFX(DeviceIds.kFrontLeftDriveMotor, "CANivore2158");
        angleMotors[0].setInverted(true);
        driveMotors[0].setInverted(true);

        // Front Right
        angleMotors[1] = new TalonFX(DeviceIds.kFrontRightAngleMotor, "CANivore2158");
        driveMotors[1] = new TalonFX(DeviceIds.kFrontRightDriveMotor, "CANivore2158");
        angleMotors[1].setInverted(true);
        driveMotors[1].setInverted(false);

        // Back Left
        angleMotors[2] = new TalonFX(DeviceIds.kBackLeftAngleMotor, "CANivore2158");
        driveMotors[2] = new TalonFX(DeviceIds.kBackLeftDriveMotor, "CANivore2158");
        angleMotors[2].setInverted(true);
        driveMotors[2].setInverted(true);

        // Back Right
        angleMotors[3] = new TalonFX(DeviceIds.kBackRightAngleMotor, "CANivore2158");
        driveMotors[3] = new TalonFX(DeviceIds.kBackRightDriveMotor, "CANivore2158");
        angleMotors[3].setInverted(true);
        driveMotors[3].setInverted(false);
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        // Get joystick inputs with deadband
        double xInput = Math.pow(applyDeadband(translationX.getAsDouble()), 3);
        double yInput = Math.pow(applyDeadband(translationY.getAsDouble()), 3);
        double rotInput = Math.pow(applyDeadband(rotation.getAsDouble()), 3);
        
        // Scale inputs to correct speeds
        double xSpeed = xInput * SystemSpeeds.kMaxDriveSpeedMetersPerSecond;
        double ySpeed = yInput * SystemSpeeds.kMaxDriveSpeedMetersPerSecond;
        double rot = rotInput * SystemSpeeds.kMaxRotationSpeedRadiansPerSecond;
        
        // Create chassis speeds (field or robot relative)
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, swerve.getGyroAngle());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
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
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i], Rotation2d.fromDegrees(currentAngle));
            
            // Angle control with continuous wrapping
            double targetAngle = moduleStates[i].angle.getDegrees();
            double angleError = targetAngle - currentAngle;
            
            // Wrap error to -180 to 180
            angleError = angleError % 360;
            if (angleError > 180) angleError -= 360;
            if (angleError < -180) angleError += 360;
            
            // Apply simple P control for angle with limiting
            double angleOutput = angleError * kP;
            angleOutput = Math.max(-maxOutput, Math.min(maxOutput, angleOutput));
            
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