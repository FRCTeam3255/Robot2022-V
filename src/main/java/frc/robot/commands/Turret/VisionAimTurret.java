// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AimState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class VisionAimTurret extends CommandBase {

  Turret subTurret;
  Vision subVision;
  double position;

  boolean precedence;

  public VisionAimTurret(Turret subTurret, Vision subVision) {
    this.subTurret = subTurret;
    this.subVision = subVision;
    addRequirements(subTurret);

    precedence = false;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    position = subTurret.getAngle() - subVision.limelight.getOffsetX();

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
      subTurret.setAngle(position);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (precedence) {
      subTurret.setSpeed(0);
      RobotContainer.aimState = AimState.NONE;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
