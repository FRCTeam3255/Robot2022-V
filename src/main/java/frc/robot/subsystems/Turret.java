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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTurret;
import frc.robot.RobotMap.mapTurret;
import frc.robot.RobotPreferences.prefTurret;

public class Turret extends SubsystemBase {

  TalonFX turretMotor;
  TalonFXConfiguration config;

  boolean displayOnDashboard;

  public Turret() {

    turretMotor = new TalonFX(mapTurret.TURRET_MOTOR_CAN);
    config = new TalonFXConfiguration();

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    config.slot0.kP = prefTurret.turretP.getValue();
    config.slot0.kI = prefTurret.turretI.getValue();
    config.slot0.kD = prefTurret.turretD.getValue();

    config.slot0.allowableClosedloopError = SN_Math
        .degreesToFalcon(prefTurret.turretAllowableClosedloopErrorDegrees.getValue(), constTurret.GEAR_RATIO);
    config.slot0.closedLoopPeakOutput = prefTurret.turretClosedLoopPeakOutput.getValue();

    turretMotor.configFactoryDefault();

    turretMotor.configAllSettings(config);

    turretMotor.setInverted(constTurret.INVERTED);

    turretMotor.configForwardSoftLimitEnable(true);
    turretMotor.configForwardSoftLimitThreshold(
        SN_Math.degreesToFalcon(prefTurret.turretMaxDegrees.getValue(), constTurret.GEAR_RATIO));
    turretMotor.configReverseSoftLimitEnable(true);
    turretMotor.configReverseSoftLimitThreshold(
        SN_Math.degreesToFalcon(prefTurret.turretMinDegrees.getValue(), constTurret.GEAR_RATIO));

    turretMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Sets the turret angle in degrees. Uses positional closed loop control.
   * 
   * @param degrees Degree count to set turret to
   */
  public void setAngle(double degrees) {
    double position = SN_Math.degreesToFalcon(degrees, constTurret.GEAR_RATIO);
    turretMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward,
        prefTurret.turretArbitraryFeedForward.getValue());
  }

  /**
   * Sets the turret angle in degrees. Uses positional closed loop control.
   * 
   * @param degrees Degree count to set turret to
   */
  public void setAngle(SN_DoublePreference degrees) {
    double position = SN_Math.degreesToFalcon(degrees.getValue(), constTurret.GEAR_RATIO);
    turretMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward,
        prefTurret.turretArbitraryFeedForward.getValue());
  }

  /**
   * Gets the turret angle in degrees
   * 
   * @return Turret angle in degrees
   */
  public double getAngle() {
    return SN_Math.falconToDegrees(turretMotor.getSelectedSensorPosition(), constTurret.GEAR_RATIO);
  }

  public void setSpeed(double speed) {
    turretMotor.set(ControlMode.PercentOutput, speed);
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

      SmartDashboard.putNumber("Turret Angle Degrees", getAngle());
      SmartDashboard.putNumber("Turret Motor Speed", turretMotor.getMotorOutputPercent());

    }

  }
}
