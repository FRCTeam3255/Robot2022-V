// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPreferences.prefAuto;
import frc.robot.commands.Cargo.CollectCargo;
import frc.robot.commands.Cargo.ShootCargo;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;

public class FourBallA extends SequentialCommandGroup {

  Drivetrain subDrivetrain;
  Shooter subShooter;
  Turret subTurret;
  Hood subHood;
  Transfer subTransfer;
  Intake subIntake;

  public FourBallA(Drivetrain subDrivetrain, Shooter subShooter, Turret subTurret, Hood subHood, Transfer subTransfer,
      Intake subIntake) {
    this.subDrivetrain = subDrivetrain;
    this.subShooter = subShooter;
    this.subTurret = subTurret;
    this.subHood = subHood;
    this.subTransfer = subTransfer;
    this.subIntake = subIntake;

    addCommands(

        parallel(
            new CollectCargo(subIntake, subTransfer)
                .until(() -> subTransfer.isTopBallCollected() && subTransfer.isBottomBallCollected()),
            new InstantCommand(() -> subShooter.setMotorRPM(prefAuto.shooterRPM.getValue())), // set shooter
            new InstantCommand(() -> subTurret.setAngle(prefAuto.turretAngle.getValue())), // set turret
            new InstantCommand(() -> subHood.setAngleDegrees(prefAuto.hoodAngle))), // set hood

        new ShootCargo(subShooter, subTransfer).withTimeout(3),

        parallel(
            new CollectCargo(subIntake, subTransfer)
                .until(() -> subTransfer.isTopBallCollected() && subTransfer.isBottomBallCollected()),
            new InstantCommand(() -> subShooter.setMotorRPM(prefAuto.shooterRPM.getValue())), // set shooter
            new InstantCommand(() -> subTurret.setAngle(prefAuto.turretAngle.getValue())), // set turret
            new InstantCommand(() -> subHood.setAngleDegrees(prefAuto.hoodAngle)), // set hood
            new InstantCommand(() -> subDrivetrain.resetPose(subDrivetrain.TRAJ_T1toB1thenB2.getInitialPose()))
                .andThen(new InstantCommand(() -> subDrivetrain.driveSpeed(0, 0)))),

        new ShootCargo(subShooter, subTransfer).withTimeout(3)

    );
  }

  @Override
  public void end(boolean interrupted) {
    subShooter.setMotorRPM(0);
    new CollectCargo(subIntake, subTransfer).end(true);
  }

}