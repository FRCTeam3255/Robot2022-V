// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.frcteam3255.utils.SN_Lerp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AimState;
import frc.robot.Constants.constHood;
import frc.robot.Constants.constShooter;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class OdometrySetShooter extends CommandBase {

  Drivetrain subDrivetrain;
  Shooter subShooter;
  Hood subHood;

  double goalRPM;
  double angle;

  double distance;

  SN_Lerp tyDistanceTable = constVision.tyDistanceTable;
  SN_Lerp distanceVelocityTable = constShooter.distanceVelocityTable;
  SN_Lerp distanceAngleTable = constHood.distanceAngleTable;

  boolean precedence;

  public OdometrySetShooter(Drivetrain subDrivetrain, Shooter subShooter, Hood subHood) {

    this.subDrivetrain = subDrivetrain;
    this.subShooter = subShooter;
    this.subHood = subHood;

    goalRPM = 0;
    angle = 0;
    distance = 0;

    precedence = false;

    addRequirements();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    distance = subDrivetrain.getDistanceFromHub();

    goalRPM = distanceVelocityTable.getOutput(distance);
    angle = distanceAngleTable.getOutput(distance);

    switch (RobotContainer.aimState) {
      case MANUAL:
        precedence = false;
        break;
      case VISION:
        precedence = false;
        break;
      case ODOMETRY:
        precedence = true;
      case NONE:
        precedence = true;
      default:
        break;
    }

    if (precedence) {
      RobotContainer.aimState = AimState.ODOMETRY;
      subShooter.setGoalRPM(goalRPM);
      subHood.setAngle(angle);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (precedence) {
      RobotContainer.aimState = AimState.NONE;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
