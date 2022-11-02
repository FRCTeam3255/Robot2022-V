// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_DualActionStick;
import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.joystick.SN_SwitchboardStick;
import com.frcteam3255.preferences.SN_Preferences;
import com.frcteam3255.utils.SN_InstantCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AimState;
import frc.robot.Constants.CargoState;
import frc.robot.Constants.constField;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefPreset;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.commands.Auto.Defense;
import frc.robot.commands.Auto.FourBallA;
import frc.robot.commands.Cargo.CollectCargo;
import frc.robot.commands.Cargo.DiscardCargo;
import frc.robot.commands.Cargo.ShootCargo;
import frc.robot.commands.Climber.MoveClimber;
import frc.robot.commands.Shooter.OdometrySetShooter;
import frc.robot.commands.Shooter.VisionSetShooter;
import frc.robot.commands.Turret.MoveTurret;
import frc.robot.commands.Turret.OdometryAimTurret;
import frc.robot.commands.Turret.VisionAimTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivetrain.AutoPath;

public class RobotContainer {

  // Controllers
  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER_CONTROLLER);
  private final SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(
      prefDrivetrain.driveSlewRateLimit.getValue());
  private final SN_DualActionStick conOperator = new SN_DualActionStick(mapControllers.OPERATOR_CONTROLLER);
  private final SN_SwitchboardStick conSwitchboard = new SN_SwitchboardStick(mapControllers.SWITCHBOARD);

  // Subsystems
  private final Drivetrain subDrivetrain = new Drivetrain();
  private final Hood subHood = new Hood();
  private final Intake subIntake = new Intake();
  private final Shooter subShooter = new Shooter();
  private final Transfer subTransfer = new Transfer();
  private final Turret subTurret = new Turret();
  private final Climber subClimber = new Climber();
  private final Vision subVision = new Vision();

  // Commands
  private final ShootCargo comShootCargo = new ShootCargo(subShooter, subTransfer);
  private final CollectCargo comCollectCargo = new CollectCargo(subIntake, subTransfer);
  private final DiscardCargo comDiscardCargo = new DiscardCargo(subIntake, subTransfer);

  private final MoveTurret comMoveTurret = new MoveTurret(subTurret, conOperator);
  private final VisionAimTurret comVisionAimTurret = new VisionAimTurret(subTurret, subVision);
  private final OdometryAimTurret comOdometryAimTurret = new OdometryAimTurret(subTurret, subDrivetrain);

  private final VisionSetShooter comVisionSetShooter = new VisionSetShooter(subShooter, subHood, subVision);
  private final OdometrySetShooter comOdometrySetShooter = new OdometrySetShooter(subDrivetrain, subShooter, subHood);

  private final MoveClimber comMoveClimber = new MoveClimber(subClimber, subTurret, conDriver);
  // Autos
  private final FourBallA autoFourBallA = new FourBallA(
      subDrivetrain,
      subShooter,
      subTurret,
      subHood,
      subTransfer,
      subIntake);

  private final Defense autoDefence = new Defense(
      subDrivetrain,
      subIntake,
      subTransfer,
      subShooter,
      subTurret,
      subHood);

  private final RamseteCommand test = subDrivetrain.getRamseteCommand(subDrivetrain.getTrajectory(AutoPath.T4toB3));
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static CargoState cargoState;
  public static AimState aimState;

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new RunCommand(
            () -> subDrivetrain.arcadeDrive(
                driveSlewRateLimiter.calculate(conDriver.getArcadeMove()), conDriver.getArcadeRotate()),
            subDrivetrain));

    subClimber.setDefaultCommand(comMoveClimber);

    cargoState = CargoState.NONE;
    aimState = AimState.NONE;

    configureButtonBindings();
    configureDashboardButtons();
    configureAutoSelector();
  }

  private void configureButtonBindings() {

    // Driver Commands

    // Driving
    conDriver.btn_LBump
        .whenPressed(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedLow))
        .whenReleased(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedMid));
    conDriver.btn_RBump
        .whenPressed(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedHigh))
        .whenReleased(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedMid));

    // Climbing
    conDriver.btn_A
        .whenPressed(() -> subClimber.setAngled());

    conDriver.btn_B
        .whenPressed(() -> subClimber.setPerpendicular());

    // Prep Climb
    conDriver.btn_Back
        .whenPressed(() -> subShooter.neutralOutput())
        .whenPressed(() -> subTurret.setAngle(prefTurret.turretMinDegrees))
        .whenPressed(() -> subHood.neutralOutput());

    // Pose resetting
    conDriver.POV_North.whenPressed(
        () -> subDrivetrain
            .resetPose(constField.LEFT_FENDER_POSITION_FRONT));
    conDriver.POV_South.whenPressed(
        () -> subDrivetrain
            .resetPose(constField.LEFT_FENDER_POSITION_BACK));
    conDriver.POV_West.whenPressed(
        () -> subDrivetrain
            .resetPose(constField.RIGHT_FENDER_POSITION_FRONT));
    conDriver.POV_East.whenPressed(
        () -> subDrivetrain
            .resetPose(constField.RIGHT_FENDER_POSITION_BACK));

    // Operator Commands

    // Shooting
    conOperator.btn_RTrig
        .whileHeld(comShootCargo)
        .whileHeld(() -> subShooter.setMotorRPMToGoalRPM())
        .whenReleased(() -> subShooter.neutralOutput());

    conOperator.btn_RBump.whenPressed(() -> subShooter.setMotorRPMToGoalRPM());

    // Turret
    conOperator.btn_LBump.whileHeld(comMoveTurret);
    conOperator.btn_LStick.whenPressed(() -> subTurret.setAngle(prefTurret.turretFacingTowardsIntakeDegrees));
    conOperator.btn_RStick.whenPressed(() -> subTurret.setAngle(prefTurret.turretFacingAwayFromIntakeDegrees));

    conOperator.btn_X
        .and(conSwitchboard.btn_1)
        .whileActiveContinuous(comVisionSetShooter);

    conOperator.btn_X
        .and(conSwitchboard.btn_3)
        .whileActiveContinuous(comVisionAimTurret);

    // Intake
    conOperator.btn_LTrig.whileHeld(comCollectCargo);
    conOperator.btn_B.whileHeld(comDiscardCargo);
    conOperator.btn_Back.whenPressed(() -> subIntake.setRetracted());

    // Presets
    conOperator.POV_North
        .whenPressed(() -> subShooter.setGoalRPM(prefPreset.presetFenderShooterRPM))
        .whenPressed(() -> subHood.setAngle(prefPreset.presetFenderHoodDegrees));

    conOperator.POV_South
        .whenPressed(() -> subShooter.setGoalRPM(prefPreset.presetLaunchpadShooterRPM))
        .whenPressed(() -> subHood.setAngle(prefPreset.presetLaunchpadHoodDegrees));

    conOperator.POV_West
        .whenPressed(() -> subShooter.setGoalRPM(prefPreset.presetTarmacShooterRPM))
        .whenPressed(() -> subHood.setAngle(prefPreset.presetTarmacHoodDegrees));

    conSwitchboard.btn_2.whileHeld(comOdometrySetShooter);
    conSwitchboard.btn_4.whileHeld(comOdometryAimTurret);

    conSwitchboard.btn_8.whileHeld(() -> subDrivetrain.resetPose(
        subVision.calculatePoseFromVision(subVision.getDistanceFromHub(),
            subDrivetrain.getPose().getRotation().getRadians(),
            Rotation2d.fromDegrees(subTurret.getAngle()).getRadians(),
            Rotation2d.fromDegrees(-subVision.limelight.getOffsetX()).getRadians())));

  }

  public void useSwitchboardButtons() {

    // btn_1 -> Vision Aim Y
    // btn_2 -> Odometry Aim Y
    // btn_3 -> Vision Aim X
    // btn_4 -> Odometry Aim X

    // btn_9 -> Use Hardcoded or Default Preference Values
    if (conSwitchboard.btn_9.get()) {
      SN_Preferences.usePreferences();
    } else {
      SN_Preferences.useDefaults();
    }

    // btn_10 -> Don't Send Values to SmartDashboard
    if (!conSwitchboard.btn_10.get()) {
      subDrivetrain.displayValuesOnDashboard();
      subHood.displayValuesOnDashboard();
      subIntake.displayValuesOnDashboard();
      subShooter.displayValuesOnDashboard();
      subTransfer.displayValuesOnDashboard();
      subTurret.displayValuesOnDashboard();
    } else {
      subDrivetrain.hideValuesOnDashboard();
      subHood.hideValuesOnDashboard();
      subIntake.hideValuesOnDashboard();
      subShooter.hideValuesOnDashboard();
      subTransfer.hideValuesOnDashboard();
      subTurret.hideValuesOnDashboard();
    }
  }

  private void configureDashboardButtons() {

    SmartDashboard.putData(
        "Configure Climber", new SN_InstantCommand(subClimber::configure, true, subClimber));
    SmartDashboard.putData(
        "Configure Drivetrain", new SN_InstantCommand(subDrivetrain::configure, true, subDrivetrain));
    SmartDashboard.putData(
        "Configure Hood", new SN_InstantCommand(subHood::configure, true, subHood));
    SmartDashboard.putData(
        "Configure Intake", new SN_InstantCommand(subIntake::configure, true, subIntake));
    SmartDashboard.putData(
        "Configure Shooter", new SN_InstantCommand(subShooter::configure, true, subShooter));
    SmartDashboard.putData(
        "Configure Transfer", new SN_InstantCommand(subTransfer::configure, true, subTransfer));
    SmartDashboard.putData(
        "Configure Turret", new SN_InstantCommand(subTurret::configure, true, subTurret));
  }

  private void configureAutoSelector() {
    autoChooser.setDefaultOption("null", null);
    autoChooser.addOption("Four Ball A", autoFourBallA);
    autoChooser.addOption("Defense", autoDefence);
    autoChooser.addOption("LEFT_FENDER_POSITION_FRONT",
        new InstantCommand(() -> subDrivetrain.resetPose(constField.LEFT_FENDER_POSITION_FRONT)));
    autoChooser.addOption("LEFT_FENDER_POSITION_BACK",
        new InstantCommand(() -> subDrivetrain.resetPose(constField.LEFT_FENDER_POSITION_BACK)));
    autoChooser.addOption("RIGHT_FENDER_POSITION_FRONT",
        new InstantCommand(() -> subDrivetrain.resetPose(constField.RIGHT_FENDER_POSITION_FRONT)));
    autoChooser.addOption("RIGHT_FENDER_POSITION_BACK",
        new InstantCommand(() -> subDrivetrain.resetPose(constField.RIGHT_FENDER_POSITION_BACK)));
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
