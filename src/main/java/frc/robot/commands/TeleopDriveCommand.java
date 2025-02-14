package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.hardware.Controller.DriverController;

import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DriverController driverController;

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem, DriverController driverController) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Begin drive command");
    }

    @Override
    public void execute() {
        switch (DriveConstants.driveType) {
            case ARCADE:
                break;
            case SWERVE:
                swerveSubsystem.swerveDriveTeleop(driverController);
                break;
            case SPINDRIVE:
                swerveSubsystem.spinDriveMotors(0.05);
                break;
            case SPINANGLE:
                swerveSubsystem.spinAngleMotors(0.3);
                break;
            case SPIN:
                swerveSubsystem.spin(1);
                break;
            case DRIVE:
                swerveSubsystem.drive(1);
                break;
            case ALIGN:
                break;
            default:
                break;
        }
        swerveSubsystem.updateOdometer();
    } 

    @Override
    public void end(boolean interrupted) {
        System.out.println("End drive command");
    }

    public boolean isFinished() {
        return false;
    }
}


