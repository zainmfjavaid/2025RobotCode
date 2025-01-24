package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Controller.DriverController;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DriverController controller;

    public DriveCommand(DriveSubsystem subsystem, DriverController controller) {
        driveSubsystem = subsystem;
        this.controller = controller;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Begin drive command");
    }

    @Override
    public void execute() {
        switch (DriveConstants.driveType) {
            case ARCADE:
                driveSubsystem.arcadeDrive(getLeftStickY(), getRightStickX());
                break;
            case SWERVE:
                driveSubsystem.swerveDriveTeleopRelativeSpeeds(getLeftStickX(), getLeftStickY(), controller.getRightStickX());
                break;
            case DRIVE:
                driveSubsystem.drive();
                break;
            case SPIN:
                driveSubsystem.spin();
                break;
            case ALIGN:
                driveSubsystem.swerveDriveSpeeds(0, 0, 0);
                break;
            default:
                break;
        }
        driveSubsystem.updateOdometer();
    } 

    public double getLeftStickX() {
        return controller.getLeftStickX();
    }
    public double getLeftStickY() {
        return controller.getLeftStickY();
    }
    public double getRightStickX() {
        return controller.getRightStickX();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("End drive command");
    }

    // PRINT
    public void printJoystickAxes() {
        System.out.println("JOYSTICK AXES");
        System.out.println("LX: " + controller.getLeftStickX());
        System.out.println("LY: " + controller.getLeftStickY());
        System.out.println("RX: " + controller.getRightStickX());
        System.out.println("RY: " + controller.getRightStickY());
    }

    public boolean isFinished() {
        return false;
    }
}


