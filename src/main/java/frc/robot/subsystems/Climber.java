// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.components.SN_DoubleSolenoid;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.prefClimber;

public class Climber extends SubsystemBase {
  
  //TODO: check if these are good names
  TalonFX climberMotor;
  SN_DoubleSolenoid pivotPiston;
  TalonFXConfiguration config;

  DigitalInput minSwitch;
  DigitalInput maxSwitch;

  boolean displayOnDashboard;

  public Climber() {
    climberMotor = new TalonFX(mapClimber.CLIMBER_MOTOR_CAN);

    minSwitch = new DigitalInput(mapClimber.CLIMBER_MINIMUM_SWITCH_DIO);
    maxSwitch = new DigitalInput(mapClimber.CLIMBER_MAXIMUM_SWITCH_DIO);

    pivotPiston = new SN_DoubleSolenoid(mapClimber.CLIMBER_PCM, PneumaticsModuleType.CTREPCM,
    mapClimber.PIVOT_PISTON_SOLENOID_PCM_A, mapClimber.PIVOT_PISTON_SOLENOID_PCM_B);

    config = new TalonFXConfiguration();
    configure();
    
    displayOnDashboard = true;

  }

  public void configure() {
    config.slot0.kP = prefClimber.climberP.getValue();
    config.slot0.kI = prefClimber.climberI.getValue();
    config.slot0.kD = prefClimber.climberD.getValue();

    config.slot0.closedLoopPeakOutput = prefClimber.climberClosedLoopSpeed.getValue();    
    config.slot0.allowableClosedloopError = SN_Math.RPMToVelocity(
      
    prefClimber.climberAllowableClosedLoopError.getValue(), SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT);
      
    climberMotor.configFactoryDefault();
    climberMotor.configAllSettings(config);

    climberMotor.configReverseSoftLimitEnable(true);
    climberMotor.configReverseSoftLimitThreshold(prefClimber.climberPerpendicularMinPos.getValue());
    climberMotor.configForwardSoftLimitEnable(true);
    climberMotor.configForwardSoftLimitThreshold(prefClimber.climberMaxPosition.getValue());
    
    climberMotor.setInverted(constClimber.INVERTED);
    climberMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}