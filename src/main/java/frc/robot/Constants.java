// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.frcteam3255.utils.SN_Lerp;
import com.frcteam3255.utils.SN_Point2D;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {

  // ALL LENGTH MEASUREMENTS MUST BE IN METERS

  public static final class constClimber {
    public static final boolean INVERTED = false;
  }

  public static final class constDrivetrain {

    public static final boolean LEFT_INVERTED = true;
    public static final boolean RIGHT_INVERTED = false;

    public static final double GEAR_RATIO = 6; // 6 motor rotations per 1 wheel rotation
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double WHEELBASE = 0.67; // length
    public static final double TRACKWIDTH = 0.55; // width
    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH);

    public static final Path PATH_T1toB1thenB2 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/T1toB1thenB2.wpilib.json");
    public static final Path PATH_B2toB4andB5 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/B2toB4andB5.wpilib.json");
    public static final Path PATH_B4toB2 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/B4toB2.wpilib.json");

  }

  public static final class constField {

    public static final Translation2d HUB_POSITION = new Translation2d(
        Units.inchesToMeters(324), Units.inchesToMeters(162));

  }

  public static final class constHood {

    public static final double GEAR_RATIO = 69.33333;
    public static final boolean INVERTED = true;

    // distance (x): meters from center of hub
    // angle (y): angle of hood to make shot from given distance
    // relies on shooter table
    private static final SN_Point2D[] distanceAnglePoints = {
        new SN_Point2D(0, 0),
        new SN_Point2D(1, 1)
    };

    public static final SN_Lerp distanceAngleTable = new SN_Lerp(distanceAnglePoints);

  }

  public static final class constIntake {

    public static final boolean ROLLER_INVERTED = true;
    public static final boolean DEPLOY_INVERTED = true;

  }

  public static final class constShooter {

    public static final boolean INVERTED = false;

    // distance (x): meters from center of hub
    // velocity (y): rpm of shooter flywheel to make shot from given distance
    // relies on hood table
    private static final SN_Point2D[] distanceVelocityPoints = {
        new SN_Point2D(0, 0),
        new SN_Point2D(1, 1)
    };

    public static final SN_Lerp distanceVelocityTable = new SN_Lerp(distanceVelocityPoints);

  }

  public static final class constTransfer {

    public static final boolean ENTRANCE_WHEEL_INVERTED = true;

  }

  public static final class constTurret {

    public static final boolean INVERTED = false;
    public static final double GEAR_RATIO = 65;

  }

  public static final class constVision {

    // ty (x): limelight y offset in limelight native units
    // distance (y): distance in inches when limelight has given y offset
    private static final SN_Point2D[] tyDistancePoints = {
        new SN_Point2D(-15.32, 211),
        new SN_Point2D(-14.5, 199),
        new SN_Point2D(-13.33, 187),
        new SN_Point2D(-11.56, 175),
        new SN_Point2D(-8.56, 163),
        new SN_Point2D(-6.82, 151),
        new SN_Point2D(-4.74, 139),
        new SN_Point2D(-2.19, 127),
        new SN_Point2D(1.55, 115),
        new SN_Point2D(5.63, 103),
        new SN_Point2D(10.55, 91),
        new SN_Point2D(16.09, 79)
    };

    public static final SN_Lerp tyDistanceTable = new SN_Lerp(tyDistancePoints);

  }

  public enum CargoState {
    SHOOTING, COLLECTING, DISCARDING, PROCESSING, NONE
  }

}
