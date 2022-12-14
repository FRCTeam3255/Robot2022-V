// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.joystick.SN_SwitchboardStick;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Turret;

public class MoveClimber extends CommandBase {

  Climber subClimber;
  Turret subTurret;
  SN_F310Gamepad conDriver;
  SN_SwitchboardStick conSwitchboard;

  double speed;

  public MoveClimber(Climber subClimber, Turret subTurret, SN_F310Gamepad conDriver,
      SN_SwitchboardStick conSwitchboard) {
    this.subClimber = subClimber;
    this.subTurret = subTurret;
    this.conDriver = conDriver;
    this.conSwitchboard = conSwitchboard;

    speed = 0;

    addRequirements(subClimber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    speed = conDriver.getAxisRT() - conDriver.getAxisLT();

    if (!conSwitchboard.btn_6.get()) {
      speed *= subTurret.getAngle() <= prefTurret.turretClimberThreshold.getValue() ? 1 : 0;
    }

    subClimber.setClimberSpeed(speed);

  }

  @Override
  public void end(boolean interrupted) {
    subClimber.neutralMotorOutput();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
