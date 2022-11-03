// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.components.SN_Limelight;
import com.frcteam3255.components.SN_Limelight.LEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constField;
import frc.robot.Constants.constVision;

public class Vision extends SubsystemBase {

  public SN_Limelight limelight;

  int buttonTimerLoops;

  public Vision() {
    limelight = new SN_Limelight();
  }

  public void setLEDOn() {
    limelight.setLEDMode(LEDMode.on);
  }

  public void setLEDOff() {
    limelight.setLEDMode(LEDMode.off);
  }

  /**
   * Get distance from hub using ty inputed to the lerp
   * 
   * @return Distance from limelight lens to center of hub
   */
  public double getDistanceFromHub() {
    return constVision.tyDistanceTable.getOutput(limelight.getOffsetY());
  }

  /**
   * Calculate Robot Position from Vision target. Specific to the Rapid React
   * field.
   * <p>
   * Radian measures require counterclockwise rotation to be positive.
   * 
   * @param distanceFromHub Distance from hub in meters
   * @param robotAngle      Robot angle in radians
   * @param turretAngle     Turret angle in radians
   * @param offsetToTarget  Angle offset to target in radians
   * @return Calculated position of the robot
   */
  public Pose2d calculatePoseFromVision(
      double distanceFromHub,
      double robotAngle,
      double turretAngle,
      double offsetToTarget) {

    double goalAngle = robotAngle + turretAngle + offsetToTarget;

    double calculatedRobotXPosition = constField.HUB_POSITION.getX() - (distanceFromHub * Math.cos(goalAngle));
    double calculatedRobotYPosition = constField.HUB_POSITION.getY() + (distanceFromHub * Math.sin(goalAngle));

    return new Pose2d(calculatedRobotXPosition, calculatedRobotYPosition, new Rotation2d(-robotAngle));
  }

  @Override
  public void periodic() {

    buttonTimerLoops++;
    if (RobotController.getUserButton() && buttonTimerLoops > 25) {
      limelight.toggleLEDs();
      buttonTimerLoops = 0;
    }
  }
}
