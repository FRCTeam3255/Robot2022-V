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
    public static final Path PATH_F1toB1 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/F1toB1.wpilib.json");
    public static final Path PATH_T4toB3 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/T4toB3.wpilib.json");
    public static final Path PATH_B3toRB3toB3 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/B3toRB3toB3two.wpilib.json");
    public static final Path PATH_TESTAUTO = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/TestPath.wpilib.json");
    public static final Path PATH_T3toTaxi = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/T3toTaxi.wpilib.json");
    public static final Path PATH_T2toB2 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/T2toB2.wpilib.json");
    public static final Path PATH_FendertoB2 = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/FendertoB2.wpilib.json");
    public static final Path PATH_B2toFender = Filesystem.getDeployDirectory().toPath()
        .resolve("pathplanner/generatedJSON/B2toFender.wpilib.json");
  }

  public static final class constField {

    public static final Translation2d HUB_POSITION = new Translation2d(
        Units.inchesToMeters(324), Units.inchesToMeters(162));

    // front refers to the intake side of robot facing in the fender
    // left and right is relative to the drivers station
    // only positions for allied alliance color, opposing alliance is not included
    public static final Pose2d LEFT_FENDER_POSITION_FRONT = new Pose2d(
        6.98, 4.61, Rotation2d.fromDegrees(-21));
    public static final Pose2d LEFT_FENDER_POSITION_BACK = new Pose2d(
        6.98, 4.61, Rotation2d.fromDegrees(159));

    public static final Pose2d RIGHT_FENDER_POSITION_FRONT = new Pose2d(
        7.78, 2.84, Rotation2d.fromDegrees(69));
    public static final Pose2d RIGHT_FENDER_POSITION_BACK = new Pose2d(
        7.78, 2.84, Rotation2d.fromDegrees(-111));

  }

  public static final class constHood {

    public static final double GEAR_RATIO = 69.33333;
    public static final boolean INVERTED = true;

    // ty (x): limelight y offset
    // angle (y): angle of hood to make shot from given ty
    // relies on shooter table
    private static final SN_Point2D[] tyAnglePoints = {
        new SN_Point2D(-17.46, 37.0),
        new SN_Point2D(-17.16, 37.0),
        new SN_Point2D(-13.33, 37.0),
        new SN_Point2D(-10.6, 35.0),
        new SN_Point2D(-6.82, 34.0),
        new SN_Point2D(-4.21, 31.0),
        new SN_Point2D(1.55, 28.0),
        new SN_Point2D(6.79, 24.0),
        new SN_Point2D(16.09, 20.0)
    };

    // distance (x): distance to center of hub from limelight lens in meters
    // angle (y): angle of hood to make shot from given distance
    // relies on shooter table
    private static final SN_Point2D[] distanceAnglePoints = {
        new SN_Point2D(Units.inchesToMeters(79.0), 20.0),
        new SN_Point2D(Units.inchesToMeters(97.0), 24.0),
        new SN_Point2D(Units.inchesToMeters(115.0), 28.0),
        new SN_Point2D(Units.inchesToMeters(133.0), 31.0),
        new SN_Point2D(Units.inchesToMeters(151.0), 34.0),
        new SN_Point2D(Units.inchesToMeters(169.0), 35.0),
        new SN_Point2D(Units.inchesToMeters(187.0), 37.0),
        new SN_Point2D(Units.inchesToMeters(229.0), 38.0)
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
        new SN_Point2D(-17.46, 4800.0),
        // new SN_Point2D(-17.16, 5300.0), (point looks REALLY weird)
        new SN_Point2D(-13.33, 3800.0),
        new SN_Point2D(-10.6, 3650.0),
        new SN_Point2D(-6.82, 3500.0),
        new SN_Point2D(-4.21, 3350.0),
        new SN_Point2D(1.55, 3255.0),
        new SN_Point2D(6.79, 3100.0),
        new SN_Point2D(16.09, 3200.0),
    };

    // distance (x): distance from hub in meters
    // velocity (y): rpm of shooter flywheel
    // relies on hood table
    private static final SN_Point2D[] distanceVelocityPoints = {
        new SN_Point2D(Units.inchesToMeters(79.0), 3200.0),
        new SN_Point2D(Units.inchesToMeters(97.0), 3100.0),
        new SN_Point2D(Units.inchesToMeters(115.0), 3255.0),
        new SN_Point2D(Units.inchesToMeters(133.0), 3350.0),
        new SN_Point2D(Units.inchesToMeters(151.0), 3500.0),
        new SN_Point2D(Units.inchesToMeters(169.0), 3650.0),
        new SN_Point2D(Units.inchesToMeters(187.0), 3800.0),
        new SN_Point2D(Units.inchesToMeters(229.0), 5300.0),
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
        new SN_Point2D(-17.16, Units.inchesToMeters(229.0)),
        new SN_Point2D(-15.32, Units.inchesToMeters(211.0)),
        new SN_Point2D(-14.5, Units.inchesToMeters(199.0)),
        new SN_Point2D(-13.33, Units.inchesToMeters(187.0)),
        new SN_Point2D(-11.56, Units.inchesToMeters(175.0)),
        new SN_Point2D(-10.6, Units.inchesToMeters(169.0)),
        new SN_Point2D(-8.56, Units.inchesToMeters(163.0)),
        new SN_Point2D(-6.82, Units.inchesToMeters(151.0)),
        new SN_Point2D(-4.74, Units.inchesToMeters(139.0)),
        new SN_Point2D(-4.21, Units.inchesToMeters(133.0)),
        new SN_Point2D(-2.19, Units.inchesToMeters(127.0)),
        new SN_Point2D(1.55, Units.inchesToMeters(115.0)),
        new SN_Point2D(5.63, Units.inchesToMeters(103.0)),
        new SN_Point2D(6.79, Units.inchesToMeters(97.0)),
        new SN_Point2D(10.55, Units.inchesToMeters(91.0)),
        new SN_Point2D(16.09, Units.inchesToMeters(79.0)),
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
