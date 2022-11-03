// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Cargo.CollectCargo;
import frc.robot.commands.Cargo.DiscardCargo;
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

public class Defense extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;
  Turret subTurret;
  Hood subHood;

  Trajectory tarmacToBallTraj;
  RamseteCommand tarmacToBall;

  Trajectory circleTraj;
  RamseteCommand circle;

  double aimTimeout = 1;
  double shootTimeout = 2;

  double tarmacToBallTime;
  double circleTime;

  public Defense(
      Drivetrain subDrivetrain,
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

    tarmacToBallTraj = subDrivetrain.getTrajectory(AutoPath.T4toB3);
    tarmacToBall = subDrivetrain.getRamseteCommand(tarmacToBallTraj);

    circleTraj = subDrivetrain.getTrajectory(AutoPath.B3toRB3toB3);
    circle = subDrivetrain.getRamseteCommand(circleTraj);

    tarmacToBallTime = tarmacToBallTraj.getTotalTimeSeconds();
    circleTime = circleTraj.getTotalTimeSeconds();

    // reset pose
    // aim with odometry
    // shoot ball
    // drive to get next ball while aiming with odometry
    // shoot that ball
    // drive to get red ball while aiming with odometry
    // discard red ball

    addCommands(
        new InstantCommand(() -> subDrivetrain.resetPose(tarmacToBallTraj.getInitialPose())), // reset pose

        parallel(
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(aimTimeout), // aim
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(aimTimeout)), // aim

        new InstantCommand(() -> subShooter.setMotorRPMToGoalRPM()), // aim
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout), // shoot

        parallel(
            new InstantCommand(() -> subDrivetrain.resetPose(tarmacToBallTraj.getInitialPose())).andThen(tarmacToBall), // drive
            new CollectCargo(subIntake, subTransfer).withTimeout(tarmacToBallTime), // collect cargo
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(tarmacToBallTime), // aim
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(tarmacToBallTime)), // aim

        new InstantCommand(() -> subShooter.setMotorRPMToGoalRPM()), // aim
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout), // shoot
        new InstantCommand(() -> subShooter.neutralOutput()), // turn off shooter

        parallel(
            new InstantCommand(() -> subDrivetrain.resetPose(circleTraj.getInitialPose())).andThen(circle), // drive
            new CollectCargo(subIntake, subTransfer).withTimeout(circleTime)), // collect cargo

        new DiscardCargo(subIntake, subTransfer)); // discard cargo
  }
}
