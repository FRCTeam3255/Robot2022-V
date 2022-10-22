// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cargo;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotPreferences;
import frc.robot.Constants.CargoState;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

public class ShootCargo extends CommandBase {

  Shooter subShooter;
  Transfer subTransfer;

  SN_DoublePreference outputEntrance;
  SN_DoublePreference outputBottom;
  SN_DoublePreference outputTop;

  public ShootCargo(Shooter subShooter, Transfer subTransfer) {

    this.subShooter = subShooter;
    this.subTransfer = subTransfer;

    addRequirements(this.subShooter);

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    RobotContainer.cargoState = CargoState.SHOOTING;

    outputEntrance = prefTransfer.transferEntranceSpeed;
    outputBottom = prefTransfer.transferBeltSpeed;
    outputTop = prefTransfer.transferBeltSpeed;

    if (!subShooter.isMotorAtSpeed()) {

      if (subTransfer.isTopBallCollected()) {
        outputTop = RobotPreferences.zeroDoublePref;

        if (subTransfer.isBottomBallCollected()) {

          outputEntrance = RobotPreferences.zeroDoublePref;
          outputBottom = RobotPreferences.zeroDoublePref;
        }
      }
    }

    subTransfer.setEntranceWheelSpeed(outputEntrance);
    subTransfer.setBottomBeltSpeed(outputBottom);
    subTransfer.setTopBeltSpeed(outputTop);
  }

  @Override
  public void end(boolean interrupted) {
    subTransfer.setEntranceWheelSpeed(RobotPreferences.zeroDoublePref);
    subTransfer.setBottomBeltSpeed(RobotPreferences.zeroDoublePref);
    subTransfer.setTopBeltSpeed(RobotPreferences.zeroDoublePref);
    RobotContainer.cargoState = CargoState.NONE;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
