// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {

  // ALL LENGTH MEASUREMENTS MUST BE IN METERS

  public static final class constClimber {
    public static final boolean INVERTED = false;
  }

  public static final class constDrivetrain {

    public static final boolean LEFT_INVERTED = false;

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

  public static final class constHood {

    public static final double GEAR_RATIO = 69.33333;
    public static final boolean INVERTED = true;

  }

  public static final class constIntake {

    public static final boolean ROLLER_INVERTED = true;
    public static final boolean DEPLOY_INVERTED = true;

  }

  public static final class constShooter {

    public static final boolean INVERTED = false;

  }

  public static final class constTransfer {

    public static final boolean ENTRANCE_WHEEL_INVERTED = true;

  }

  public static final class constTurret {

    public static final boolean INVERTED = false;
    public static final double GEAR_RATIO = 65;

  }

  public enum CargoState {
    SHOOTING, COLLECTING, DISCARDING, PROCESSING, NONE
  }

}
