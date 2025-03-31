// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoncommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.commands.elevator.ElevatorScore;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LevelFourUpAndDown extends SequentialCommandGroup {
  /** Creates a new A. */
  public LevelFourUpAndDown(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LevelFourUp(elevatorSubsystem, intakeSubsystem), 
      new ElevatorScore(intakeSubsystem, elevatorSubsystem, IntakeState.L4)
    );
  }
}
