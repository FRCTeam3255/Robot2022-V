// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class VisionResetPose extends CommandBase {
  Drivetrain subDrivetrain;
  Vision subVision;
  Turret subTurret;

  Pose2d calculatedPose;

  double distanceFromHub;
  double robotAngle;
  double turretAngle;
  double offsetToTarget;

  public VisionResetPose(Drivetrain subDrivetrain, Vision subVision, Turret subTurret) {
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;
    this.subTurret = subTurret;

    calculatedPose = new Pose2d();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    distanceFromHub = subVision.getDistanceFromHub();
    robotAngle = subDrivetrain.getPose().getRotation().getRadians();
    turretAngle = Units.degreesToRadians(subTurret.getAngle());
    offsetToTarget = Units.degreesToRadians(-subVision.limelight.getOffsetX());

    calculatedPose = subVision.calculatePoseFromVision(distanceFromHub, robotAngle, turretAngle, offsetToTarget);

    if (subVision.limelight.hasTarget()) {
      subDrivetrain.resetPose(calculatedPose);
    }

    boolean deadTest = subTurret.getAngle() > prefTurret.turretDeadzoneLarge.getValue()
        || subTurret.getAngle() < prefTurret.turretDeadzoneSmall.getValue();
    SmartDashboard.putBoolean("!!!deadzone", deadTest);

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
