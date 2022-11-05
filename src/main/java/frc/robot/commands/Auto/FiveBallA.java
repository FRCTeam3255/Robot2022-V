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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallA extends SequentialCommandGroup {
  Drivetrain subDrivetrain;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;
  Turret subTurret;
  Hood subHood;

  Trajectory T2toB2Traj;
  RamseteCommand T2toB2;
  double T2toB2Time;

  Trajectory FendertoB2Traj;
  RamseteCommand FendertoB2;
  double FendertoB2Time;

  Trajectory B2toFenderTraj;
  RamseteCommand B2toFender;
  double B2toFenderTime;

  double aimTimeout = 1;
  double shootTimeout = 2;

  public FiveBallA(Drivetrain subDrivetrain,
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

    T2toB2Traj = subDrivetrain.getTrajectory(AutoPath.T2toB2);
    T2toB2 = subDrivetrain.getRamseteCommand(T2toB2Traj);
    T2toB2Time = T2toB2Traj.getTotalTimeSeconds();

    FendertoB2Traj = subDrivetrain.getTrajectory(AutoPath.FendertoB2);
    FendertoB2 = subDrivetrain.getRamseteCommand(FendertoB2Traj);
    FendertoB2Time = FendertoB2Traj.getTotalTimeSeconds();

    B2toFenderTraj = subDrivetrain.getTrajectory(AutoPath.B2toFender);
    B2toFender = subDrivetrain.getRamseteCommand(B2toFenderTraj);
    B2toFenderTime = B2toFenderTraj.getTotalTimeSeconds();

    // reset pose
    // aim (odemetry)
    // shoot
    // drive (T2toB2) and collect
    // aim & shoot
    // drive (B2toFender but its actually terminal) and collect
    // drive BACK (Fender to B2 but its still just terminal)
    // aim and shoot (with vision????)

    addCommands(
        // reset pose
        new InstantCommand(() -> subDrivetrain.resetPose(T2toB2Traj.getInitialPose())),

        // aim
        parallel(
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(aimTimeout),
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(aimTimeout)),

        // shoot
        new InstantCommand(() -> subShooter.setMotorRPMToGoalRPM()),
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout),

        // drive t2 b2, collect, aim
        parallel(
            new InstantCommand(() -> subDrivetrain.resetPose(T2toB2Traj.getInitialPose())).andThen(T2toB2),
            new CollectCargo(subIntake, subTransfer).withTimeout(T2toB2Time),
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(T2toB2Time),
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(T2toB2Time),
            new RunCommand(() -> subShooter.setMotorRPMToGoalRPM()).withTimeout(T2toB2Time)),

        // shoot
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout),
        new InstantCommand(() -> subShooter.neutralOutput()),

        // drive and collect
        parallel(
            new InstantCommand(() -> subDrivetrain.resetPose(B2toFenderTraj.getInitialPose())).andThen(B2toFender),
            new CollectCargo(subIntake, subTransfer).withTimeout(B2toFenderTime)),

        // drive and aim
        parallel(
            new InstantCommand(() -> subDrivetrain.resetPose(FendertoB2Traj.getInitialPose())).andThen(FendertoB2),
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(FendertoB2Time),
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(FendertoB2Time)),

        // shoot
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout));
  }
}
