// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import com.frcteam3255.utils.SN_Lerp;
import com.frcteam3255.utils.SN_Point2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final Pose2d LEFT_FENDER_POSITION = new Pose2d(
        new Translation2d(9.5, 3.8), new Rotation2d(Units.degreesToRadians(-21)));

  }

  public static final class constHood {

    public static final double GEAR_RATIO = 69.33333;
    public static final boolean INVERTED = true;

    // ty (x): limelight y offset
    // angle (y): angle of hood to make shot from given ty
    // relies on shooter table
    private static final SN_Point2D[] tyAnglePoints = {
        new SN_Point2D(-13.33, 37.0),
        new SN_Point2D(-6.82, 34.0),
        new SN_Point2D(1.55, 28.0),
        new SN_Point2D(16.09, 20.0)
    };

    private static final SN_Point2D[] distanceAnglePoints = {
        new SN_Point2D(Units.inchesToMeters(79.0), 20.0),
        new SN_Point2D(Units.inchesToMeters(115.0), 28.0),
        new SN_Point2D(Units.inchesToMeters(151.0), 34.0),
        new SN_Point2D(Units.inchesToMeters(187.0), 37.0)
    };

    public static final SN_Lerp tyAngleTable = new SN_Lerp(tyAnglePoints);

    public static final SN_Lerp distanceAngleTable = new SN_Lerp(distanceAnglePoints);

  }

  public static final class constIntake {

    public static final boolean ROLLER_INVERTED = true;
    public static final boolean DEPLOY_INVERTED = true;

  }

  public static final class constShooter {

    public static final boolean INVERTED = false;

    // ty (x): y offset of limelight
    // velocity (y): rpm of shooter flywheel to make shot
    // relies on hood table

    private static final SN_Point2D[] tyVelocityPoints = {
        new SN_Point2D(-13.33, 3800),
        new SN_Point2D(-6.82, 3500),
        new SN_Point2D(1.55, 3255),
        new SN_Point2D(16.09, 3200)
    };

    // distance (x): distance from hub in meters
    // velocity (y): rpm of shooter flywheel
    private static final SN_Point2D[] distanceVelocityPoints = {
        new SN_Point2D(Units.inchesToMeters(79.0), 3200.0),
        new SN_Point2D(Units.inchesToMeters(151.0), 3500.0),
        new SN_Point2D(Units.inchesToMeters(115.0), 3255.0),
        new SN_Point2D(Units.inchesToMeters(187.0), 3800.0)

    };

    public static final SN_Lerp tyVelocityTable = new SN_Lerp(tyVelocityPoints);

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
    // distance (y): distance in meters when limelight has given y offset
    private static final SN_Point2D[] tyDistancePoints = {
        new SN_Point2D(Units.inchesToMeters(-15.32), 211),
        new SN_Point2D(Units.inchesToMeters(-14.5), 199),
        new SN_Point2D(Units.inchesToMeters(-13.33), 187),
        new SN_Point2D(Units.inchesToMeters(-11.56), 175),
        new SN_Point2D(Units.inchesToMeters(-8.56), 163),
        new SN_Point2D(Units.inchesToMeters(-6.82), 151),
        new SN_Point2D(Units.inchesToMeters(-4.74), 139),
        new SN_Point2D(Units.inchesToMeters(-2.19), 127),
        new SN_Point2D(Units.inchesToMeters(1.55), 115),
        new SN_Point2D(Units.inchesToMeters(5.63), 103),
        new SN_Point2D(Units.inchesToMeters(10.55), 91),
        new SN_Point2D(Units.inchesToMeters(16.09), 79)
    };

    public static final SN_Lerp tyDistanceTable = new SN_Lerp(tyDistancePoints);

  }

  public enum CargoState {
    SHOOTING, COLLECTING, DISCARDING, PROCESSING, NONE
  }

  public enum AimState {
    MANUAL, VISION, ODOMETRY, NONE
  }
}
