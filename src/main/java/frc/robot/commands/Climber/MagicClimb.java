// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MagicClimb extends SequentialCommandGroup {
  Climber subClimber;

  public MagicClimb(Climber subClimber) {
    this.subClimber = subClimber;

    addCommands(
        // Will remove these comments after the code looks good (i am going insane)

        // Run the climber all the way to the bottom (grab the rung)
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberPerpendicularMinPos))
            .until(() -> subClimber.getMinSwitch()),
        // Move up until we're at a position where we can angle (I'd prefer to not use a
        // timer for this because having the
        // encoder counts reset at the bottom and then using those seems better)
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberOptimalAnglingPosition))
            .until(() -> subClimber.canAngle()),
        // Pivot. Keeping this instant because I don't see how it isn't instant
        new InstantCommand(() -> subClimber.setPivoted()),
        // Run the climber all the way up
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberAngledMaxPos))
            .until(() -> subClimber.getMaxSwitch()));
  }
}
