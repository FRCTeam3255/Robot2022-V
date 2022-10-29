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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CargoState;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefPreset;
import frc.robot.RobotPreferences.prefTurret;
import frc.robot.commands.Auto.FourBallA;
import frc.robot.commands.Cargo.CollectCargo;
import frc.robot.commands.Cargo.DiscardCargo;
import frc.robot.commands.Cargo.ShootCargo;
import frc.robot.commands.Climber.MoveClimber;
import frc.robot.commands.Turret.MoveTurret;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

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

  private final MoveTurret comMoveTurret = new MoveTurret(subTurret, subClimber, conOperator);

  private final MoveClimber comMoveClimber = new MoveClimber(subClimber, subTurret, conDriver);
  // Autos
  private final FourBallA autoFourBallA = new FourBallA(subDrivetrain);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static CargoState cargoState;

  public RobotContainer() {

    subDrivetrain.setDefaultCommand(
        new RunCommand(
            () -> subDrivetrain.arcadeDrive(
                driveSlewRateLimiter.calculate(conDriver.getArcadeMove()), conDriver.getArcadeRotate()),
            subDrivetrain));

    subClimber.setDefaultCommand(comMoveClimber);

    cargoState = CargoState.NONE;

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
    conDriver.btn_Back
        .whenPressed(() -> subTurret.setAngle(prefTurret.turretMinDegrees))
        .whenPressed(() -> subHood.neutralOutput());

    // Operator Commands

    // Shooting
    conOperator.btn_RTrig
        .whileHeld(comShootCargo)
        .whileHeld(() -> subShooter.setMotorRPMToGoalRPM())
        .whenReleased(() -> subShooter.neutralOutput());

    conOperator.btn_RBump.whenPressed(() -> subShooter.setMotorRPMToGoalRPM());

    // Turret
    conOperator.btn_LBump.whileHeld(comMoveTurret);
    conOperator.btn_LStick.and(new Trigger(subClimber::isMinSwitch))
        .whenActive(() -> subTurret.setAngle(prefTurret.turretFacingTowardsIntakeDegrees));
    conOperator.btn_RStick.and(new Trigger(subClimber::isMinSwitch))
        .whenActive(() -> subTurret.setAngle(prefTurret.turretFacingAwayFromIntakeDegrees));

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

    // Switchboard Commands

    // btn_1 -> Send Values to SmartDashboard
    conSwitchboard.btn_1
        .whenPressed(() -> subDrivetrain.displayValuesOnDashboard())
        .whenPressed(() -> subHood.displayValuesOnDashboard())
        .whenPressed(() -> subIntake.displayValuesOnDashboard())
        .whenPressed(() -> subShooter.displayValuesOnDashboard())
        .whenPressed(() -> subTransfer.displayValuesOnDashboard())
        .whenPressed(() -> subTurret.displayValuesOnDashboard());
    conSwitchboard.btn_1
        .whenReleased(() -> subDrivetrain.hideValuesOnDashboard())
        .whenReleased(() -> subHood.hideValuesOnDashboard())
        .whenReleased(() -> subIntake.hideValuesOnDashboard())
        .whenReleased(() -> subShooter.hideValuesOnDashboard())
        .whenReleased(() -> subTransfer.hideValuesOnDashboard())
        .whenReleased(() -> subTurret.hideValuesOnDashboard());

    // btn_2 -> Use Hardcoded or Default Preference Values
    conSwitchboard.btn_2
        .whenPressed(() -> SN_Preferences.usePreferences())
        .whenReleased(() -> SN_Preferences.useDefaults());

  }

  private void configureDashboardButtons() {

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
    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
