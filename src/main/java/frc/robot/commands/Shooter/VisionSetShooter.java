// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.frcteam3255.components.SN_Limelight;
import com.frcteam3255.utils.SN_Lerp;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AimState;
import frc.robot.Constants.constHood;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class VisionSetShooter extends CommandBase {

  Shooter subShooter;
  Hood subHood;
  Vision subVision;
  Turret subTurret;

  double goalRPM;
  double angle;

  SN_Lerp tyVelocityTable = constShooter.tyVelocityTable;
  SN_Lerp tyAngleTable = constHood.tyAngleTable;

  SN_Limelight limelight;

  boolean precedence;

  public VisionSetShooter(Shooter subShooter, Hood subHood, Vision subVision, Turret subTurret) {
    this.subShooter = subShooter;
    this.subHood = subHood;
    this.subVision = subVision;
    this.subTurret = subTurret;

    limelight = subVision.limelight;

    goalRPM = 0;
    angle = 0;

    precedence = false;

    addRequirements();

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    goalRPM = tyVelocityTable.getOutput(limelight.getOffsetY());
    angle = tyAngleTable.getOutput(limelight.getOffsetY());

    switch (RobotContainer.aimState) {
      case MANUAL:
        precedence = false;
        break;
      case VISION:
        precedence = true;
        break;
      case ODOMETRY:
        precedence = true;
      case NONE:
        precedence = true;
      default:
        break;
    }

    if (precedence) {
      RobotContainer.aimState = AimState.VISION;
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
