// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;
import com.frcteam3255.preferences.SN_ZeroDoublePreference;
import com.frcteam3255.preferences.SN_ZeroIntPreference;

public final class RobotPreferences {
  public static final SN_ZeroIntPreference zeroIntPref = new SN_ZeroIntPreference();
  public static final SN_ZeroDoublePreference zeroDoublePref = new SN_ZeroDoublePreference();

  public static final class prefAuto {
    public static final SN_DoublePreference shooterRPM = new SN_DoublePreference("shooterRPM", 1); // Sets RPM for
                                                                                                   // shooter
    public static final SN_DoublePreference turretAngle = new SN_DoublePreference("turretAngle", 0); // Sets turret
                                                                                                     // angle for auto
    public static final SN_DoublePreference hoodAngle = new SN_DoublePreference("hoodAngle", 0); // Sets hood angle for
                                                                                                 // auto
  }

  public static final class prefClimber {
    public static final SN_DoublePreference climberArbitraryFeedForward = new SN_DoublePreference(
        "climberArbitraryFeedForward", 0);
    public static final SN_DoublePreference climberP = new SN_DoublePreference("climberP", 1);
    public static final SN_DoublePreference climberI = new SN_DoublePreference("climberI", 0);
    public static final SN_DoublePreference climberD = new SN_DoublePreference("climberD", 0);

    public static final SN_DoublePreference climberClosedLoopSpeed = new SN_DoublePreference(
        "climberClosedLoopSpeed", 1);
    public static final SN_DoublePreference climberAllowableClosedLoopError = new SN_DoublePreference(
        "climberAllowableClosedLoopError", 0);

    public static final SN_DoublePreference climberPerpendicularMinPos = new SN_DoublePreference(
        "climberPerpendicularMinPos", 0);
    public static final SN_DoublePreference climberAngledMinPos = new SN_DoublePreference("climberAngledMinPos",
        200000);
    public static final SN_DoublePreference climberPerpendicularMaxPos = new SN_DoublePreference(
        "climberPerpendicularMaxPos", 277300);
    public static final SN_DoublePreference climberAngledMaxPos = new SN_DoublePreference("climberAngledMaxPos",
        277300);
  }

  public static final class prefDrivetrain {

    public static final SN_DoublePreference driveF = new SN_DoublePreference("driveF", 0.045);
    public static final SN_DoublePreference driveP = new SN_DoublePreference("driveP", 0.1);
    public static final SN_DoublePreference driveI = new SN_DoublePreference("driveI", 0);
    public static final SN_DoublePreference driveD = new SN_DoublePreference("driveD", 0);

    public static final SN_DoublePreference driveArcadeSpeedHigh = new SN_DoublePreference("driveArcadeSpeedHigh",
        1);
    public static final SN_DoublePreference driveArcadeSpeedMid = new SN_DoublePreference("driveArcadeSpeedMid",
        0.65);
    public static final SN_DoublePreference driveArcadeSpeedLow = new SN_DoublePreference("driveArcadeSpeedLow",
        0.25);
    public static final SN_DoublePreference driveArcadeTurnMid = new SN_DoublePreference("driveArcadeTurn", 0.7);

    public static final SN_DoublePreference driveSlewRateLimit = new SN_DoublePreference("driveSlewRateLimit", 2);

  }

  public static final class prefHood {

    public static final SN_DoublePreference hoodArbitraryFeedForward = new SN_DoublePreference(
        "hoodArbitraryFeedForward", 0.040078);

    public static final SN_DoublePreference hoodP = new SN_DoublePreference("hoodP", 0.09);
    public static final SN_DoublePreference hoodI = new SN_DoublePreference("hoodI", 0);
    public static final SN_DoublePreference hoodD = new SN_DoublePreference("hoodD", 0);

    public static final SN_DoublePreference hoodAllowableClosedLoopErrorDegrees = new SN_DoublePreference(
        "hoodAllowableClosedLoopErrorDegrees", 0.0001);
    public static final SN_DoublePreference hoodClosedLoopPeakOutput = new SN_DoublePreference(
        "hoodClosedLoopPeakOutput", .25);

    public static final SN_DoublePreference hoodMinDegrees = new SN_DoublePreference("hoodMinDegrees", 4.89);
    public static final SN_DoublePreference hoodMaxDegrees = new SN_DoublePreference("hoodMaxDegrees", 38);

    public static final SN_DoublePreference hoodNudgeDegrees = new SN_DoublePreference("hoodNudgeDegrees", 1);

  }

  public static final class prefIntake {

    public static final SN_DoublePreference intakeRollerSpeed = new SN_DoublePreference("intakeRollerSpeed", 0.8);

  }

  public static final class prefPreset {

    public static final SN_DoublePreference presetFenderShooterRPM = new SN_DoublePreference(
        "presetFenderShooterRPM", 2700);
    public static final SN_DoublePreference presetFenderHoodDegrees = new SN_DoublePreference(
        "presetFenderHoodDegrees", 7);

    public static final SN_DoublePreference presetTarmacShooterRPM = new SN_DoublePreference(
        "presetTarmacShooterRPM", 3255);
    public static final SN_DoublePreference presetTarmacHoodDegrees = new SN_DoublePreference(
        "presetTarmacHoodDegrees", 15);

    public static final SN_DoublePreference presetLaunchpadShooterRPM = new SN_DoublePreference(
        "presetLaunchpadShooterRPM", 4200);
    public static final SN_DoublePreference presetLaunchpadHoodDegrees = new SN_DoublePreference(
        "presetLaunchpadHoodDegrees", 38);

  }

  public static final class prefShooter {

    public static final SN_DoublePreference shooterF = new SN_DoublePreference("shooterF", 0);
    public static final SN_DoublePreference shooterP = new SN_DoublePreference("shooterP", 0.02);
    public static final SN_DoublePreference shooterI = new SN_DoublePreference("shooterI", 0.0002);
    public static final SN_DoublePreference shooterD = new SN_DoublePreference("shooterD", 0.0088);

    public static final SN_DoublePreference shooterAllowableClosedloopErrorRPM = new SN_DoublePreference(
        "shooterAllowableClosedloopErrorRPM", 15);

  }

  public static final class prefTransfer {

    public static final SN_DoublePreference transferEntranceSpeed = new SN_DoublePreference(
        "transferEntranceSpeed", 0.8);
    public static final SN_DoublePreference transferEntranceReverseSpeed = new SN_DoublePreference(
        "transferEntranceReverseSpeed", -0.8);
    public static final SN_DoublePreference transferBeltSpeed = new SN_DoublePreference(
        "transferBeltSpeed", 0.5);
    public static final SN_DoublePreference transferBeltReverseSpeed = new SN_DoublePreference(
        "transferBeltReverseSpeed", -0.5);

  }

  public static final class prefTurret {

    public static final SN_DoublePreference turretArbitraryFeedForward = new SN_DoublePreference(
        "turretArbitraryFeedForward", 0);

    public static final SN_DoublePreference turretP = new SN_DoublePreference("turretP", 0.15);
    public static final SN_DoublePreference turretI = new SN_DoublePreference("turretI", 0.00001);
    public static final SN_DoublePreference turretD = new SN_DoublePreference("turretD", 8);

    public static final SN_DoublePreference turretAllowableClosedloopErrorDegrees = new SN_DoublePreference(
        "turretAllowableClosedloopErrorDegrees", 3);
    public static final SN_DoublePreference turretClosedLoopPeakOutput = new SN_DoublePreference(
        "turretClosedLoopPeakOutput", 1);

    public static final SN_DoublePreference turretOpenLoopSpeed = new SN_DoublePreference("turretOpenLoopSpeed",
        0.3);

    public static final SN_DoublePreference turretMinDegrees = new SN_DoublePreference("turretMinDegrees", -270);
    public static final SN_DoublePreference turretMaxDegrees = new SN_DoublePreference("turretMaxDegrees", 110);
    public static final SN_DoublePreference turretFacingTowardsIntakeDegrees = new SN_DoublePreference(
        "turretFacingTowardsIntakeDegrees", -90);
    public static final SN_DoublePreference turretFacingAwayFromIntakeDegrees = new SN_DoublePreference(
        "turretFacingAwayFromIntakeDegrees", 90);

  }

}
