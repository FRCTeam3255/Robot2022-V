// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {

  TalonFX leadMotor;
  TalonFX followMotor;
  TalonFXConfiguration config;

  double goalRPM;

  boolean displayOnDashboard;

  public Shooter() {

    leadMotor = new TalonFX(mapShooter.LEAD_MOTOR_CAN);
    followMotor = new TalonFX(mapShooter.FOLLOW_MOTOR_CAN);

    config = new TalonFXConfiguration();
    configure();

    displayOnDashboard = true;

    goalRPM = 0;
  }

  public void configure() {
    config.slot0.kF = prefShooter.shooterF.getValue();
    config.slot0.kP = prefShooter.shooterP.getValue();
    config.slot0.kI = prefShooter.shooterI.getValue();
    config.slot0.kD = prefShooter.shooterD.getValue();

    config.slot0.integralZone = prefShooter.shooterIZone.getValue();

    leadMotor.configFactoryDefault();
    followMotor.configFactoryDefault();

    leadMotor.configAllSettings(config);

    leadMotor.setInverted(constShooter.INVERTED);
    followMotor.setInverted(InvertType.OpposeMaster);

    leadMotor.setNeutralMode(NeutralMode.Coast);

    leadMotor.configClosedloopRamp(prefShooter.shooterClosedLoopRamp.getValue());

    followMotor.follow(leadMotor);
  }

  public void setMotorRPM(double rpm) {
    double velocity = SN_Math.RPMToVelocity(rpm, SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);
    leadMotor.set(ControlMode.Velocity, velocity);
  }

  public void neutralOutput() {
    leadMotor.neutralOutput();
  }

  public double getMotorRPM() {
    return SN_Math.velocityToRPM(leadMotor.getSelectedSensorVelocity(), SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);
  }

  public boolean isMotorAtSpeed() {
    return SN_Math.velocityToRPM(leadMotor.getClosedLoopError(),
        SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT) < prefShooter.shooterAllowableClosedloopErrorRPM.getValue();
  }

  public void setGoalRPM(double goalRPM) {
    this.goalRPM = goalRPM;
  }

  public void setGoalRPM(SN_DoublePreference goalRPM) {
    setGoalRPM(goalRPM.getValue());
  }

  public double getGoalRPM() {
    return goalRPM;
  }

  public void setMotorRPMToGoalRPM() {
    setMotorRPM(getGoalRPM());
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

      SmartDashboard.putNumber("Shooter Motor RPM", getMotorRPM());
      SmartDashboard.putNumber("Shooter Motor Percent Output", leadMotor.getMotorOutputPercent());
      SmartDashboard.putNumber("Shooter Goal RPM", getGoalRPM());
      SmartDashboard.putBoolean("Shooter Is At Speed", isMotorAtSpeed());

      SmartDashboard.putNumber("Shooter Native Velocity", leadMotor.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Shooter Native Error", leadMotor.getClosedLoopError());
      SmartDashboard.putNumber("Shooter Native Target", leadMotor.getClosedLoopTarget());

    }

  }
}
