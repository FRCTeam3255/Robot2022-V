// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

public final class RobotPreferences {

  public static final class prefDrivetrain {

    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0);

    public static final SN_DoublePreference driveArcadeSpeedHigh = new SN_DoublePreference("driveArcadeSpeedHigh", 1);
    public static final SN_DoublePreference driveArcadeSpeedMid = new SN_DoublePreference("driveArcadeSpeedMid", 0.65);
    public static final SN_DoublePreference driveArcadeSpeedLow = new SN_DoublePreference("driveArcadeSpeedLow", 0.25);
    public static final SN_DoublePreference driveArcadeTurnMid = new SN_DoublePreference("driveArcadeTurn", 0.7);

    public static final SN_DoublePreference driveSlewRateLimit = new SN_DoublePreference("driveSlewRateLimit", 2);
  }

}
