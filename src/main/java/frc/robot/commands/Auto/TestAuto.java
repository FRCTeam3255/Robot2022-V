// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.AutoPath;

public class TestAuto extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Trajectory testTraj;
  RamseteCommand test;

  public TestAuto(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;
    testTraj = subDrivetrain.getTrajectory(AutoPath.TestAuto);
    test = subDrivetrain.getRamseteCommand(testTraj);
    addCommands(
        new InstantCommand(() -> subDrivetrain.resetPose(testTraj.getInitialPose())),
        test);
  }
}
