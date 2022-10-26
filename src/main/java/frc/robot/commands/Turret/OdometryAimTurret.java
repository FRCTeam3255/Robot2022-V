// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constField;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class OdometryAimTurret extends CommandBase {

  Turret subTurret;
  Drivetrain subDrivetrain;
  Vision subVision;

  // Pose2d is position (x, y) and rotation (theta)
  Pose2d robotPose;
  // Translation2d is position (x, y)
  Translation2d hubPosition;

  Translation2d adjustedHubPosition;

  // angle robot needs to turn to face hub, if robot angle was 0 (facing right)
  double fieldRelativeAngleToHubRadians;
  // angle robot needs to turn to face hub
  double robotRelativeAngleToHubRadians;
  // angle turret needs to turn to face hub
  double outputToTurretDegrees;

  public OdometryAimTurret(Turret subTurret, Drivetrain subDrivetrain, Vision subVision) {
    this.subTurret = subTurret;
    this.subDrivetrain = subDrivetrain;
    this.subVision = subVision;

    robotPose = new Pose2d(0, 0, new Rotation2d(0));
    hubPosition = constField.HUB_POSITION;
    adjustedHubPosition = new Translation2d(0, 0);

    addRequirements();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    robotPose = subDrivetrain.getPose();

    // maintain relative position between robot and hub, but put robot at the origin
    // and have the goal orbit the origin
    adjustedHubPosition = new Translation2d(
        hubPosition.getX() - robotPose.getX(),
        hubPosition.getY() - robotPose.getY());

    fieldRelativeAngleToHubRadians = Math.atan2(adjustedHubPosition.getY(), adjustedHubPosition.getX());
    robotRelativeAngleToHubRadians = fieldRelativeAngleToHubRadians - (robotPose.getRotation().getRadians());

    // turret zero is 90 degrees off from robot zero, hence the - 90
    outputToTurretDegrees = Units.radiansToDegrees(robotRelativeAngleToHubRadians) - 90;

    // if the calculated angle goes out of the turret's range, correct for it
    while (outputToTurretDegrees > prefTurret.turretMaxDegrees.getValue()) {
      outputToTurretDegrees -= 360;
    }
    while (outputToTurretDegrees < prefTurret.turretMinDegrees.getValue()) {
      outputToTurretDegrees += 360;
    }

    subTurret.setAngle(outputToTurretDegrees);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
