// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Cargo.CollectCargo;
import frc.robot.commands.Cargo.ShootCargo;
import frc.robot.commands.Shooter.OdometrySetShooter;
import frc.robot.commands.Turret.OdometryAimTurret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Drivetrain.AutoPath;

public class F1toB1 extends SequentialCommandGroup {
  Drivetrain subDrivetrain;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;
  Turret subTurret;
  Hood subHood;

  Trajectory F1toB1Traj;
  RamseteCommand F1toB1;
  double F1toB1Time;

  double aimTimeout = 1;
  double shootTimeout = 2;

  public F1toB1(Drivetrain subDrivetrain,
      Intake subIntake,
      Transfer subTransfer,
      Shooter subShooter,
      Turret subTurret,
      Hood subHood) {

    this.subDrivetrain = subDrivetrain;
    this.subIntake = subIntake;
    this.subTransfer = subTransfer;
    this.subShooter = subShooter;
    this.subTurret = subTurret;
    this.subHood = subHood;

    F1toB1Traj = subDrivetrain.getTrajectory(AutoPath.F1toB1);
    F1toB1 = subDrivetrain.getRamseteCommand(F1toB1Traj);
    F1toB1Time = F1toB1Traj.getTotalTimeSeconds();

    // shoot preloaded
    // move backward, intake
    // shoot 1

    addCommands(
        new InstantCommand(() -> subDrivetrain.resetPose(F1toB1Traj.getInitialPose())),

        parallel(
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(aimTimeout),
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(aimTimeout)),

        new InstantCommand(() -> subShooter.setMotorRPMToGoalRPM()),
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout),

        parallel(
            new InstantCommand(() -> subDrivetrain.resetPose(F1toB1Traj.getInitialPose())).andThen(F1toB1),
            new CollectCargo(subIntake, subTransfer).withTimeout(F1toB1Time),
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(F1toB1Time),
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(F1toB1Time),
            new RunCommand(() -> subShooter.setMotorRPMToGoalRPM()).withTimeout(F1toB1Time)),

        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout),
        new InstantCommand(() -> subShooter.neutralOutput()));
  }
}
