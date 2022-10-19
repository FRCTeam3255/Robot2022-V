// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.AutoPath;

public class FourBallA extends SequentialCommandGroup {

  Drivetrain subDrivetrain;

  public FourBallA(Drivetrain subDrivetrain) {
    this.subDrivetrain = subDrivetrain;

    Trajectory TRAJ_T1toB1thenB2 = subDrivetrain.getTrajectory(AutoPath.T1toB1thenB2);
    RamseteCommand T1toB1thenB2 = subDrivetrain.getRamseteCommand(TRAJ_T1toB1thenB2);

    addCommands(

        new InstantCommand(() -> subDrivetrain.resetPose(TRAJ_T1toB1thenB2.getInitialPose())),
        T1toB1thenB2,
        new InstantCommand(() -> subDrivetrain.driveSpeed(0, 0))

    );
  }
}
