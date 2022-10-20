// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotMap.mapTransfer;

public class Transfer extends SubsystemBase {

  TalonFX entranceWheel;
  TalonFX bottomBelt;
  TalonFX topBelt;

  DigitalInput bottomLeftSwitch;
  DigitalInput bottomRightSwitch;
  DigitalInput topLeftSwitch;
  DigitalInput topRightSwitch;

  boolean displayOnDashboard;

  public Transfer() {

    entranceWheel = new TalonFX(mapTransfer.ENTRANCE_MOTOR_CAN);
    bottomBelt = new TalonFX(mapTransfer.BOTTOM_MOTOR_CAN);
    topBelt = new TalonFX(mapTransfer.TOP_MOTOR_CAN);

    bottomLeftSwitch = new DigitalInput(mapTransfer.BOTTOM_LEFT_SWITCH_DIO);
    bottomRightSwitch = new DigitalInput(mapTransfer.BOTTOM_RIGHT_SWITCH_DIO);
    topLeftSwitch = new DigitalInput(mapTransfer.TOP_LEFT_SWITCH_DIO);
    topRightSwitch = new DigitalInput(mapTransfer.TOP_RIGHT_SWITCH_DIO);

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    entranceWheel.configFactoryDefault();
    bottomBelt.configFactoryDefault();
    topBelt.configFactoryDefault();

    entranceWheel.setNeutralMode(NeutralMode.Brake);
    bottomBelt.setNeutralMode(NeutralMode.Brake);
    topBelt.setNeutralMode(NeutralMode.Brake);

    entranceWheel.setInverted(constTransfer.ENTRANCE_WHEEL_INVERTED);
  }

  public void setTopBeltSpeed(SN_DoublePreference speed) {
    topBelt.set(ControlMode.PercentOutput, speed.getValue());
  }

  public void setBottomBeltSpeed(SN_DoublePreference speed) {
    bottomBelt.set(ControlMode.PercentOutput, speed.getValue());
  }

  public void setEntranceWheelSpeed(SN_DoublePreference speed) {
    entranceWheel.set(ControlMode.PercentOutput, speed.getValue());
  }

  public boolean isTopBallCollected() {
    return !topLeftSwitch.get() || !topRightSwitch.get();
  }

  public boolean isBottomBallCollected() {
    return !bottomLeftSwitch.get() || !bottomRightSwitch.get();
  }

  public void displayValuesOnDashboard() {
    displayOnDashboard = true;
  }

  public void hideValuesOnDashboard() {
    displayOnDashboard = false;
  }

  @Override
  public void periodic() {

    if (displayOnDashboard) {

      SmartDashboard.putNumber("Transfer Top Belt Speed", topBelt.getMotorOutputPercent());
      SmartDashboard.putNumber("Transfer Bottom Belt Speed", bottomBelt.getMotorOutputPercent());
      SmartDashboard.putNumber("Transfer Entrance Wheel Speed", entranceWheel.getMotorOutputPercent());

      SmartDashboard.putBoolean("Transfer Is Top Ball Collected", isTopBallCollected());
      SmartDashboard.putBoolean("Transfer Is Bottom Ball Collected", isBottomBallCollected());

      SmartDashboard.putBoolean("Transfer Is Top Left Switch", topLeftSwitch.get());
      SmartDashboard.putBoolean("Transfer Is Top Right Switch", topRightSwitch.get());
      SmartDashboard.putBoolean("Transfer Is Bottom Left Switch", bottomLeftSwitch.get());
      SmartDashboard.putBoolean("Transfer Is Bottom Right Switch", bottomLeftSwitch.get());

    }

  }
}
