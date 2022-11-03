// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

public class T3toTaxi extends SequentialCommandGroup {
  Drivetrain subDrivetrain;
  Intake subIntake;
  Transfer subTransfer;
  Shooter subShooter;
  Turret subTurret;
  Hood subHood;

  Trajectory T3toTaxiTraj;
  RamseteCommand T3toTaxi;

  double aimTimeout = 1;
  double shootTimeout = 2;

  public T3toTaxi(
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

    T3toTaxiTraj = subDrivetrain.getTrajectory(AutoPath.T3toTaxi);
    T3toTaxi = subDrivetrain.getRamseteCommand(T3toTaxiTraj);

    // reset pose
    // aim
    // shoot balls
    // taxi

    addCommands(
        new InstantCommand(() -> subDrivetrain.resetPose(T3toTaxiTraj.getInitialPose())), // reset pose

        parallel(
            new OdometryAimTurret(subTurret, subDrivetrain).withTimeout(aimTimeout), // aim
            new OdometrySetShooter(subDrivetrain, subShooter, subHood).withTimeout(aimTimeout)), // aim

        new InstantCommand(() -> subShooter.setMotorRPMToGoalRPM()), // aim
        new ShootCargo(subShooter, subTransfer).withTimeout(shootTimeout), // shoot

        new InstantCommand(() -> subDrivetrain.resetPose(T3toTaxiTraj.getInitialPose())), // reset pose
        T3toTaxi // taxi

    );
  }
}
