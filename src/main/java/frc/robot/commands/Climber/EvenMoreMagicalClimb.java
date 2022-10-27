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

//keeping the name funny for now so that it doesn't drive me too insane
public class EvenMoreMagicalClimb extends SequentialCommandGroup {
  Climber subClimber;

  public EvenMoreMagicalClimb(Climber subClimber) {
    this.subClimber = subClimber;

    addCommands(
        // Step 0: Hooks above the mid rung (done by the Driver)
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberPerpendicularMinPos))
            .until(() -> subClimber.getMinSwitch()), // Step 1
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberOptimalAnglingPosition))
            .until(() -> subClimber.isInOptimalAnglingRange()), // Step 2
        new InstantCommand(() -> subClimber.setAngled()), // Step 3
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberAngledMaxPos))
            .until(() -> subClimber.getMaxSwitch()), // Step 4
        new InstantCommand(() -> subClimber.setPerpendicular()), // step 5
        // INSERT THE WEIRD PART HERE (STEP 6)
        new RunCommand(() -> subClimber.setClimberPosition(prefClimber.climberAngledMaxPos))
            .until(() -> subClimber.getMaxSwitch()), // Step 7
        new InstantCommand(() -> subClimber.setAngled()), // step 8
        // WEIRD PART HERE (STEP 9)
        new InstantCommand(() -> subClimber.setPerpendicular())); // Step 10
  }
}
