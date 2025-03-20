// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;
import frc.robot.hardware.MotorController.MotorConfig;

public class SwerveSubsystem extends SubsystemBase {
    public static final double kPhysicalMaxSpeed = Units.feetToMeters(1.9); //Max drivebase speed in meters per second
    public static final double kPhysicalMaxAngularSpeed = 2 * Math.PI / 15; //Max drivebase angular speed in radians per second

    public static final double kTrackWidth = Units.inchesToMeters(21.5); //Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(21.5); //Distance between front and back wheels
    public static final double driveBaseRadius = Math.sqrt(((kTrackWidth/2) * (kTrackWidth/2)) + ((kWheelBase/2) * (kWheelBase/2)));

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( //Creates robot geometry using the locations of the 4 wheels
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
        new Translation2d(kWheelBase / 2, -kTrackWidth /2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        
    public static final double kXYTranslationP = 0;
    public static final double kRotationP = 0;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0;

    public static final double kAutoRotationP = 1.15;
    public static final double kAutoRotationI = 0;
    public static final double kAutoRotationD = 0.1;

    private final SwerveModule frontLeft = new SwerveModule(
        MotorConfig.FrontLeftModuleDrive,
        MotorConfig.FrontLeftModuleTurn,
        EncoderConfig.FrontLeftModule,
        "FL");

    private final SwerveModule frontRight = new SwerveModule(
        MotorConfig.FrontRightModuleDrive,
        MotorConfig.FrontRightModuleTurn,
        EncoderConfig.FrontRightModule,
        "FR");

    private final SwerveModule backLeft = new SwerveModule(
        MotorConfig.BackLeftModuleDrive,
        MotorConfig.BackLeftModuleTurn,
        EncoderConfig.BackLeftModule,
        "BL");

    private final SwerveModule backRight = new SwerveModule(
        MotorConfig.BackRightModuleDrive,
        MotorConfig.BackRightModuleTurn,
        EncoderConfig.BackRightModule,
        "BR");
    
    private Pigeon2 gyro = new Pigeon2(20, "CANivore2158");
    private double gyroOffset; //Offset in degrees
    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, getRotation2d(), getModulePositions());

    private ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    private GenericEntry controlOrientationEntry = matchTab.add("FOD", true).getEntry();
    private GenericEntry headingEntry = matchTab.add("NavX Yaw", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    private GenericEntry pitchEntry = matchTab.add("NavX Pitch", 0).withWidget(BuiltInWidgets.kGyro).getEntry();

    private ShuffleboardTab configTab = Shuffleboard.getTab("Config");
    private GenericEntry positionEntry = configTab.add("Position", "").getEntry();

    public boolean controlOrientationIsFOD;

    public Double rotationHold;

    private PIDController rotationController;
    private PIDController autoRotationController;

    public SwerveSubsystem() {
        zeroHeading();
        controlOrientationIsFOD = true;

        //Add coast mode command to shuffleboard
        configTab.add(new StartEndCommand(this::coastModules, this::brakeModules, this).ignoringDisable(true).withName("Coast Modules"));
        this.coastModules();
        //Define PID controllers for tracking trajectory
        rotationController = new PIDController(kRotationP, kRotationI, kAutoRotationD);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        autoRotationController = new PIDController(kAutoRotationP, kAutoRotationI, kAutoRotationD);
        autoRotationController.enableContinuousInput(-Math.PI, Math.PI);
    }   

    public void zeroHeading() {
        gyro.reset();
        gyroOffset = 0;
    }

    public void zeroHeading(Rotation2d rotation2d) {
        gyro.reset();
        gyroOffset = rotation2d.getDegrees();
    }

    public double getHeading() {
        return Math.IEEEremainder(-(gyro.getYaw().getValueAsDouble() + gyroOffset), 360);
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public void toggleOrientation(){
        //Toggle control orientation from FOD/ROD
        controlOrientationIsFOD = !controlOrientationIsFOD;
        controlOrientationEntry.setBoolean(controlOrientationIsFOD);
    }

    public void enableRotationHold(double angle){
        //Set the angle to automatically align the drive to using degrees -180 to 180
        rotationHold = Units.degreesToRadians(angle);
    }

    public double adjustSwerveAngle(double angle) {
        // Adjust angle of swerve relative to current position, i.e for aligning with a target
        rotationHold = Units.degreesToRadians(getHeading()) + Units.degreesToRadians(angle);
        return rotationHold;
    }

    public void disableRotationHold(){
        rotationHold = null;
    }

    public boolean isRotationHold() {
        if (rotationHold != null) {
            return true;
        } else {
            return false;
        }
    }

    public SwerveModuleState[] convertToModuleStates(double xTranslation, double yTranslation, double rotation) {
        //Takes axis input from joysticks and returns an array of swerve module states

        double x = yTranslation; //Intentional, x in swerve kinematics is y on the joystick
        double y = xTranslation;
        double r = rotation;

        if (Math.abs(r) > 0){
            disableRotationHold();
        }
        else if(rotationHold != null){
            r = autoRotationController.calculate(Units.degreesToRadians(getHeading()), rotationHold);
        }

        //Map to speeds in meters/radians per second
        x *= kPhysicalMaxSpeed;
        y *= kPhysicalMaxSpeed;
        r *= kPhysicalMaxAngularSpeed;

        //Construct Chassis Speeds
        ChassisSpeeds chassisSpeeds;
        if(controlOrientationIsFOD){
            //Field Oriented Drive
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, this.getRotation2d());
        } else {
            //Robot Oriented Drive
            chassisSpeeds = new ChassisSpeeds(x, y, r);
        }
        SmartDashboard.putString("chassis speeds",chassisSpeeds.toString());
        //Convert Chassis Speeds to individual module states
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        return moduleStates;
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kDriveKinematics.toSwerveModuleStates(speeds));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kPhysicalMaxSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kDriveKinematics.toChassisSpeeds(getModuleStates());
    }


    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return new ChassisSpeeds(
                getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos()
                        - getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
                getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getCos()
                        + getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
                getChassisSpeeds().omegaRadiansPerSecond);
    }

    public double getFieldRelativeRobotVy() {
        return getFieldRelativeChassisSpeeds().vyMetersPerSecond;
    }

    public double getFieldRelativeRobotVx() {
        return getFieldRelativeChassisSpeeds().vxMetersPerSecond;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveModuleArray = new SwerveModuleState[4];
        swerveModuleArray[0] = frontLeft.getState();
        swerveModuleArray[1] = frontRight.getState();
        swerveModuleArray[2] = backLeft.getState();
        swerveModuleArray[3] = backRight.getState();

        return swerveModuleArray;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] swerveModuleArray = new SwerveModulePosition[4];
        swerveModuleArray[0] = frontLeft.getPosition();
        swerveModuleArray[1] = frontRight.getPosition();
        swerveModuleArray[2] = backLeft.getPosition();
        swerveModuleArray[3] = backRight.getPosition();

        return swerveModuleArray;
    }  

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void parkModules(){
        //this tells the method in swerve module which wheels should be 45 degrees and which ones should be -45 degrees
        frontLeft.park(true);
        frontRight.park(false);
        backLeft.park(false);
        backRight.park(true);
    }

    public void coastModules(){
        frontLeft.coast();
        frontRight.coast();
        backLeft.coast();
        backRight.coast();
    }

    public void brakeModules(){
        frontLeft.brake();
        frontRight.brake();
        backLeft.brake();
        backRight.brake();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putData(this);
        odometer.update(getRotation2d(), getModulePositions());
        pitchEntry.setDouble(getPitch());
        headingEntry.setDouble(getHeading());
        positionEntry.setString(getPose().getTranslation().toString());
    }
}