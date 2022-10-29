// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constHood;
import frc.robot.RobotMap.mapHood;
import frc.robot.RobotPreferences.prefHood;

public class Hood extends SubsystemBase {

  TalonFX hoodMotor;
  DigitalInput bottomSwitch;
  TalonFXConfiguration config;

  boolean displayOnDashboard;

  public Hood() {

    hoodMotor = new TalonFX(mapHood.HOOD_MOTOR_CAN);
    bottomSwitch = new DigitalInput(mapHood.HOOD_BOTTOM_SWITCH_DIO);
    config = new TalonFXConfiguration();

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    hoodMotor.configFactoryDefault();

    config.slot0.kP = prefHood.hoodP.getValue();
    config.slot0.kI = prefHood.hoodI.getValue();
    config.slot0.kD = prefHood.hoodD.getValue();

    config.slot0.allowableClosedloopError = SN_Math
        .degreesToFalcon(prefHood.hoodAllowableClosedLoopErrorDegrees.getValue(), constHood.GEAR_RATIO);

    config.slot0.closedLoopPeakOutput = prefHood.hoodClosedLoopPeakOutput.getValue();

    hoodMotor.configAllSettings(config);

    hoodMotor.setInverted(constHood.INVERTED);
    hoodMotor.setNeutralMode(NeutralMode.Coast);

    hoodMotor.configForwardSoftLimitEnable(true);
    hoodMotor.configForwardSoftLimitThreshold(
        SN_Math.degreesToFalcon(prefHood.hoodMaxDegrees.getValue(), constHood.GEAR_RATIO));
    hoodMotor.configReverseSoftLimitEnable(true);
    hoodMotor.configReverseSoftLimitThreshold(
        SN_Math.degreesToFalcon(prefHood.hoodMinDegrees.getValue(), constHood.GEAR_RATIO));

    resetAngleToBottom();
  }

  public double getAngleDegrees() {
    return SN_Math.falconToDegrees(hoodMotor.getSelectedSensorPosition(), constHood.GEAR_RATIO);
  }

  public void setAngle(double degrees) {

    if (degrees < prefHood.hoodMinDegrees.getValue()) {
      degrees = prefHood.hoodMinDegrees.getValue();
    }

    hoodMotor.set(ControlMode.Position, SN_Math.degreesToFalcon(degrees, constHood.GEAR_RATIO),
        DemandType.ArbitraryFeedForward, prefHood.hoodArbitraryFeedForward.getValue());
  }

  public void setAngle(SN_DoublePreference degrees) {
    setAngle(degrees.getValue());
  }

  public boolean isBottomSwitch() {
    return !bottomSwitch.get();
  }

  public void resetAngleToBottom() {
    hoodMotor.setSelectedSensorPosition(
        SN_Math.degreesToFalcon(prefHood.hoodMinDegrees.getValue(), constHood.GEAR_RATIO));
  }

  public void neutralOutput() {
    hoodMotor.neutralOutput();
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

      SmartDashboard.putNumber("Hood Angle Degrees", getAngleDegrees());
      SmartDashboard.putBoolean("Hood Is Bottom Switch", isBottomSwitch());

    }

    if (isBottomSwitch()) {
      resetAngleToBottom();
    }

  }
}
