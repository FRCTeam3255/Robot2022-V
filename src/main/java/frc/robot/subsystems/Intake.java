// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.frcteam3255.components.SN_DoubleSolenoid;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {

  TalonFX roller;
  SN_DoubleSolenoid deploy;

  boolean displayOnDashboard;

  public Intake() {

    roller = new TalonFX(mapIntake.INTAKE_MOTOR_CAN);
    deploy = new SN_DoubleSolenoid(
        PneumaticsModuleType.CTREPCM, mapIntake.INTAKE_SOLENOID_PCM_FORWARD, mapIntake.INTAKE_SOLENOID_PCM_REVERSE);

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    roller.configFactoryDefault();

    roller.setInverted(constIntake.ROLLER_INVERTED);
    deploy.setInverted(constIntake.DEPLOY_INVERTED);

  }

  public void setRollerSpeed(SN_DoublePreference speed) {
    roller.set(ControlMode.PercentOutput, speed.getValue());
  }

  public void setDeployed() {
    deploy.setDeployed();
  }

  public void setRetracted() {
    deploy.setRetracted();
  }

  public boolean isDeployed() {
    return deploy.isDeployed();
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

      SmartDashboard.putNumber("Intake Roller Percent Output", roller.getMotorOutputPercent());
      SmartDashboard.putBoolean("Intake Deployed", isDeployed());

    }

  }
}
