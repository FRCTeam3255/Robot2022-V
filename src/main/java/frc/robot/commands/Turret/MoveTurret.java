// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Turret;

import com.frcteam3255.joystick.SN_DualActionStick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.AimState;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {

  Turret subTurret;
  SN_DualActionStick conOperator;
  double speed;

  public MoveTurret(Turret subTurret, SN_DualActionStick conOperator) {
    this.subTurret = subTurret;
    this.conOperator = conOperator;
    addRequirements(subTurret);
  }

  @Override
  public void initialize() {
    RobotContainer.aimState = AimState.MANUAL;
  }

  @Override
  public void execute() {
    speed = -conOperator.getRightStickX();

    subTurret.setSpeed(speed * prefTurret.turretOpenLoopSpeed.getValue());
  }

  @Override
  public void end(boolean interrupted) {
    subTurret.setSpeed(0);
    RobotContainer.aimState = AimState.NONE;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
