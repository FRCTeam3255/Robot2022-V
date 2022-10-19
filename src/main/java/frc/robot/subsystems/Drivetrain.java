// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.utils.SN_Math;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constDrivetrain;
import frc.robot.RobotMap.mapDrivetrain;
import frc.robot.RobotPreferences.prefDrivetrain;

public class Drivetrain extends SubsystemBase {

  TalonFX leftLead;
  TalonFX leftFollow;
  TalonFX rightLead;
  TalonFX rightFollow;

  AHRS navx;

  DifferentialDriveOdometry odometry;
  Field2d field = new Field2d();

  TalonFXConfiguration config;

  double arcadeDriveSpeedMultiplier;
  double arcadeDriveTurnMultiplier;
  boolean displayOnDashboard;

  public Drivetrain() {

    leftLead = new TalonFX(mapDrivetrain.LEFT_LEAD_MOTOR_CAN);
    leftFollow = new TalonFX(mapDrivetrain.LEFT_FOLLOW_MOTOR_CAN);
    rightLead = new TalonFX(mapDrivetrain.RIGHT_LEAD_MOTOR_CAN);
    rightFollow = new TalonFX(mapDrivetrain.RIGHT_FOLLOW_MOTOR_CAN);

    navx = new AHRS();

    arcadeDriveSpeedMultiplier = prefDrivetrain.driveArcadeSpeedMid.getValue();
    arcadeDriveTurnMultiplier = prefDrivetrain.driveArcadeTurnMid.getValue();

    displayOnDashboard = true;

    odometry = new DifferentialDriveOdometry(navx.getRotation2d());
    field = new Field2d();

    config = new TalonFXConfiguration();

  }

  public void configure() {
    config.slot0.kP = prefDrivetrain.driveP.getValue();
    config.slot0.kI = prefDrivetrain.driveI.getValue();
    config.slot0.kD = prefDrivetrain.driveD.getValue();

    leftLead.configFactoryDefault();
    leftFollow.configFactoryDefault();
    rightLead.configFactoryDefault();
    rightFollow.configFactoryDefault();

    leftLead.setInverted(false);
    leftFollow.setInverted(InvertType.FollowMaster);
    rightLead.setInverted(true);
    rightFollow.setInverted(InvertType.FollowMaster);

    leftLead.setSensorPhase(false);
    rightLead.setSensorPhase(true);

    leftLead.setNeutralMode(NeutralMode.Brake);
    rightLead.setNeutralMode(NeutralMode.Brake);

    leftFollow.follow(leftLead);
    rightFollow.follow(rightLead);

  }

  public double getLeftMeters() {
    double falconCounts = leftLead.getSelectedSensorPosition();
    double falconRotations = falconCounts / SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT;
    double wheelRotations = falconRotations * constDrivetrain.GEAR_RATIO;
    double meters = wheelRotations * constDrivetrain.WHEEL_CIRCUMFERENCE;

    return meters;
  }

  public double getRightMeters() {
    double falconCounts = rightLead.getSelectedSensorPosition();
    double falconRotations = falconCounts / SN_Math.TALONFX_ENCODER_PULSES_PER_COUNT;
    double wheelRotations = falconRotations * constDrivetrain.GEAR_RATIO;
    double meters = wheelRotations * constDrivetrain.WHEEL_CIRCUMFERENCE;

    return meters;
  }

  public void resetEncoderCounts() {
    leftLead.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
  }

  public double getLeftMetersPerSecond() {
    return SN_Math.falconToMPS(
        leftLead.getSelectedSensorVelocity(),
        constDrivetrain.WHEEL_CIRCUMFERENCE,
        constDrivetrain.GEAR_RATIO);
  }

  public double getRightMetersPerSecond() {
    return SN_Math.falconToMPS(
        rightLead.getSelectedSensorVelocity(),
        constDrivetrain.WHEEL_CIRCUMFERENCE,
        constDrivetrain.GEAR_RATIO);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(pose, navx.getRotation2d());
    resetEncoderCounts();
  }

  public void updatePose() {
    odometry.update(navx.getRotation2d(), getLeftMeters(), getRightMeters());
    field.setRobotPose(getPose());
  }

  public void setArcadeDriveSpeedMultiplier(SN_DoublePreference multiplier) {
    arcadeDriveSpeedMultiplier = multiplier.getValue();
  }

  public void setArcadeDriveTurnMultiplier(SN_DoublePreference multiplier) {
    arcadeDriveTurnMultiplier = multiplier.getValue();
  }

  public void arcadeDrive(double speed, double turn) {
    leftLead.set(ControlMode.PercentOutput, speed * arcadeDriveSpeedMultiplier,
        DemandType.ArbitraryFeedForward, +turn * arcadeDriveTurnMultiplier);
    rightLead.set(ControlMode.PercentOutput, speed * arcadeDriveSpeedMultiplier,
        DemandType.ArbitraryFeedForward, -turn * arcadeDriveTurnMultiplier);
  }

  public void driveSpeed(double leftMPS, double rightMPS) {

    double leftVelocity = SN_Math.MPSToFalcon(
        leftMPS, constDrivetrain.WHEEL_CIRCUMFERENCE, constDrivetrain.GEAR_RATIO);
    double rightVelocity = SN_Math.MPSToFalcon(
        rightMPS, constDrivetrain.WHEEL_CIRCUMFERENCE, constDrivetrain.GEAR_RATIO);

    leftLead.set(ControlMode.Velocity, leftVelocity);
    rightLead.set(ControlMode.Velocity, rightVelocity);

  }

  public void displayValuesOnDashboard() {
    displayOnDashboard = true;
  }

  public void hideValuesOnDashboard() {
    displayOnDashboard = false;
  }

  @Override
  public void periodic() {
    updatePose();

    if (displayOnDashboard) {

      SmartDashboard.putNumber("Drivetrain Left Encoder", leftLead.getSelectedSensorPosition());
      SmartDashboard.putNumber("Drivetrain Right Encoder", rightLead.getSelectedSensorPosition());

      SmartDashboard.putNumber("Drivetrain Left Percent Output", leftLead.getMotorOutputPercent());
      SmartDashboard.putNumber("Drivetrain Right Percent Output", rightLead.getMotorOutputPercent());

      SmartDashboard.putNumber("Drivetrain Left Velocity", leftLead.getSelectedSensorVelocity());
      SmartDashboard.putNumber("Drivetrain Right Velocity", rightLead.getSelectedSensorVelocity());

      SmartDashboard.putNumber("Drivetrain Left Meters Per Second", getLeftMetersPerSecond());
      SmartDashboard.putNumber("Drivetrain Right Meters Per Second", getRightMetersPerSecond());

      SmartDashboard.putNumber("Drivetrain Left Meters", getLeftMeters());
      SmartDashboard.putNumber("Drivetrain Right Meters", getRightMeters());

      SmartDashboard.putNumber("Drivetrain Heading Degrees", navx.getRotation2d().getDegrees());

      SmartDashboard.putNumber("Drivetrain Pose X Meters", getPose().getX());
      SmartDashboard.putNumber("Drivetrain Pose Y Meters", getPose().getY());

    }
  }
}
