// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotPreferences;
import frc.robot.RobotContainer.CargoState;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class DisgardCargo extends CommandBase {

  Intake subIntake;
  Transfer subTransfer;

  boolean precedence;

  public DisgardCargo(Intake subIntake, Transfer subTransfer) {

    this.subIntake = subIntake;
    this.subTransfer = subTransfer;

    precedence = false;

  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // if the robot is disgarding, it only takes precedence over processing
    switch (RobotContainer.cargoState) {
      case SHOOTING:
        precedence = false;
        break;
      case COLLECTING:
        precedence = false;
        break;
      case DISGARDING:
        precedence = true;
        break;
      case PROCESSING:
        precedence = true;
        break;
      case NONE:
        precedence = true;
        break;
    }

    if (precedence) {

      RobotContainer.cargoState = CargoState.DISGARDING;

      if (subIntake.isDeployed()) {
        subIntake.setRetracted();
      }

      subTransfer.setTopBeltSpeed(prefTransfer.transferBeltReverseSpeed);
      subTransfer.setEntranceWheelSpeed(prefTransfer.transferEntranceReverseSpeed);

    }
  }

  @Override
  public void end(boolean interrupted) {

    if (precedence) {

      RobotContainer.cargoState = CargoState.NONE;

      subTransfer.setTopBeltSpeed(RobotPreferences.zeroDoublePref);
      subTransfer.setEntranceWheelSpeed(RobotPreferences.zeroDoublePref);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
