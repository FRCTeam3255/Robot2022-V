// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.components.SN_Limelight;
import com.frcteam3255.components.SN_Limelight.LEDMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  @Override
  public void periodic() {

    buttonTimerLoops++;
    if (RobotController.getUserButton() && buttonTimerLoops > 25) {
      limelight.toggleLEDs();
      buttonTimerLoops = 0;
    }
  }
}
