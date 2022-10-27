// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefClimber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//keeping the name funny for now so that it doesn't drive me too insane
//also, same comments situation with magic climb
public class EvenMoreMagicalClimb extends SequentialCommandGroup {
  Climber subClimber;
  Drivetrain subDrivetrain;

  public EvenMoreMagicalClimb(Climber subClimber, Drivetrain subDrivetrain) {
    this.subClimber = subClimber;
    this.subDrivetrain = subDrivetrain;

    addCommands(
        // Step 0: Hooks above the mid rung (done by the Driver)
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberPerpendicularMinPos))
            .until(() -> subClimber.getMinSwitch()),
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberOptimalAnglingPosition))
            .until(() -> subClimber.isInOptimalAnglingRange()),
        new InstantCommand(() -> subClimber.setAngled()),
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberAngledMaxPos))
            .until(() -> subClimber.getMaxSwitch()),
        new InstantCommand(() -> subClimber.setPerpendicular()).until(() -> subDrivetrain.isOptimalSwing()),
        // my thinking here is that the good swing value would be slightly
        // below the end of the swing so that it has time to lower the climbers, if this
        // even works as intended
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberPerpendicularMinPos))
            .until(() -> subClimber.getMinSwitch()),
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberAngledMaxPos))
            .until(() -> subClimber.getMaxSwitch()),
        new InstantCommand(() -> subClimber.setAngled()).until(() -> subDrivetrain.isOptimalSwing()),
        // Same thinking here
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberPerpendicularMinPos))
            .until(() -> subClimber.getMinSwitch()),
        new InstantCommand(() -> subClimber.setPerpendicular()));
  }
}
