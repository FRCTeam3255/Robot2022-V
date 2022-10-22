// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Cargo;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotPreferences;
import frc.robot.RobotContainer.CargoState;
import frc.robot.RobotPreferences.prefIntake;
import frc.robot.RobotPreferences.prefTransfer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class CollectCargo extends CommandBase {

  Intake subIntake;
  Transfer subTransfer;

  SN_DoublePreference outputIntake;
  SN_DoublePreference outputEntrance;
  SN_DoublePreference outputBottom;
  SN_DoublePreference outputTop;

  boolean precedence;

  public CollectCargo(Intake subIntake, Transfer subTransfer) {

    this.subIntake = subIntake;
    this.subTransfer = subTransfer;

    outputIntake = prefIntake.intakeRollerSpeed;
    outputEntrance = prefTransfer.transferEntranceSpeed;
    outputBottom = prefTransfer.transferBeltSpeed;
    outputTop = prefTransfer.transferBeltSpeed;

    precedence = false;

    addRequirements(this.subIntake);

  }

  @Override
  public void initialize() {
    // don't deploy intake if there are two cargo in the robot
    if (!(subTransfer.isTopBallCollected() && subTransfer.isBottomBallCollected())) {
      subIntake.setDeployed();
    }
  }

  @Override
  public void execute() {

    outputIntake = prefIntake.intakeRollerSpeed;
    outputEntrance = prefTransfer.transferEntranceSpeed;
    outputBottom = prefTransfer.transferBeltSpeed;
    outputTop = prefTransfer.transferBeltSpeed;

    if (subTransfer.isTopBallCollected()) {
      outputTop = RobotPreferences.zeroDoublePref;

      if (subTransfer.isBottomBallCollected()) {

        outputIntake = RobotPreferences.zeroDoublePref;
        outputEntrance = RobotPreferences.zeroDoublePref;
        outputBottom = RobotPreferences.zeroDoublePref;

        if (subIntake.isDeployed()) {
          subIntake.setRetracted();
        }
      }
    }

    // if the robot is collecting, it takes precedence over everything but shooting
    switch (RobotContainer.cargoState) {
      case SHOOTING:
        precedence = false;
        break;
      case COLLECTING:
        precedence = true;
        break;
      case DISCARDING:
        precedence = true;
        break;
      case PROCESSING:
        precedence = true;
        break;
      case NONE:
        precedence = true;
        break;
    }

    // only set transfer motors if this command takes precedence
    if (precedence) {
      RobotContainer.cargoState = CargoState.COLLECTING;

      subTransfer.setEntranceWheelSpeed(outputEntrance);
      subTransfer.setBottomBeltSpeed(outputBottom);
      subTransfer.setTopBeltSpeed(outputTop);

    }

    // intake doesn't affect other commands so it gets set regardless of this
    // command taking precedence
    subIntake.setRollerSpeed(outputIntake);

  }

  @Override
  public void end(boolean interrupted) {

    subIntake.setRollerSpeed(RobotPreferences.zeroDoublePref);

    if (precedence) {
      RobotContainer.cargoState = CargoState.NONE;

      subTransfer.setEntranceWheelSpeed(RobotPreferences.zeroDoublePref);
      subTransfer.setBottomBeltSpeed(RobotPreferences.zeroDoublePref);
      subTransfer.setTopBeltSpeed(RobotPreferences.zeroDoublePref);
    }
  }

  @Override
  public boolean isFinished() {

    // once the robot has two cargo, this command will raise the intake and cease to
    // run any motors. there isn't any reason for this command to run after that
    // point, so it should end. having an end condition also makes it easier to work
    // with in auto

    return subTransfer.isTopBallCollected() && subTransfer.isBottomBallCollected();
  }
}
