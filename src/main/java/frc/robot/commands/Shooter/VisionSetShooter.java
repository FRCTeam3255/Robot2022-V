// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.frcteam3255.components.SN_Limelight;
import com.frcteam3255.utils.SN_Lerp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constHood;
import frc.robot.Constants.constShooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class VisionSetShooter extends CommandBase {

  Shooter subShooter;
  Hood subHood;
  Vision subVision;

  double goalRPM;
  double angle;

  SN_Lerp tyVelocityTable = constShooter.tyVelocityTable;
  SN_Lerp tyAngleTable = constHood.tyAngleTable;

  SN_Limelight limelight;

  public VisionSetShooter(Shooter subShooter, Hood subHood, Vision subVision) {
    this.subShooter = subShooter;
    this.subHood = subHood;
    this.subVision = subVision;

    limelight = subVision.limelight;

    goalRPM = 0;
    angle = 0;

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    goalRPM = tyVelocityTable.getOutput(limelight.getOffsetY());
    angle = tyAngleTable.getOutput(limelight.getOffsetY());

    subShooter.setGoalRPM(goalRPM);
    subHood.setAngle(angle);

    SmartDashboard.putNumber("!ty", limelight.getOffsetY());
    SmartDashboard.putNumber("!goalRPM", goalRPM);
    SmartDashboard.putNumber("!angle", angle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
