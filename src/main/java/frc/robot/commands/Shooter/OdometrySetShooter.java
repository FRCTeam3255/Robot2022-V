// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.frcteam3255.components.SN_Limelight;
import com.frcteam3255.utils.SN_Lerp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.constDrivetrain;
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

  public OdometrySetShooter(Drivetrain subDrivetrain, Shooter subShooter, Hood subHood) {

    this.subDrivetrain = subDrivetrain;
    this.subShooter = subShooter;
    this.subHood = subHood;

    goalRPM = 0;
    angle = 0;
    distance = 0;

    addRequirements(subHood);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    distance = subDrivetrain.getDistanceFromHub();

    goalRPM = distanceVelocityTable.getOutput(distance);
    angle = distanceAngleTable.getOutput(distance);

    subShooter.setGoalRPM(goalRPM);
    subHood.setAngle(angle);

    SmartDashboard.putNumber("!!distance", distance);
    SmartDashboard.putNumber("!!goalRPM", goalRPM);
    SmartDashboard.putNumber("!!angle", angle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
