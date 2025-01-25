package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.hardware.Controller;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Controller controller;

    public TeleopDriveCommand(SwerveSubsystem swerveSubsystem, Controller controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Begin drive command");
    }

    @Override
    public void execute() {
        swerveSubsystem.swerveDriveTeleop(controller);
    } 

    @Override
    public void end(boolean interrupted) {
        System.out.println("End drive command");
    }

    public boolean isFinished() {
        return false;
    }
}


