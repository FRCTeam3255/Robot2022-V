// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_DualActionStick;
import com.frcteam3255.joystick.SN_F310Gamepad;
import com.frcteam3255.joystick.SN_SwitchboardStick;
import com.frcteam3255.preferences.SN_Preferences;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.RobotPreferences.prefPreset;
import frc.robot.commands.CollectCargo;
import frc.robot.commands.ShootCargo;
import frc.robot.commands.Auto.FourBallA;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;

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

  // Commands
  private final ShootCargo comShootCargo = new ShootCargo(subShooter, subTransfer);
  private final CollectCargo comCollectCargo = new CollectCargo(subIntake, subTransfer);

  // Autos
  private final FourBallA autoFourBallA = new FourBallA(subDrivetrain);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static CargoState cargoState;

  public RobotContainer() {
    configureButtonBindings();

    subDrivetrain.setDefaultCommand(
        new RunCommand(
            () -> subDrivetrain.arcadeDrive(
                driveSlewRateLimiter.calculate(conDriver.getArcadeMove()), conDriver.getArcadeRotate()),
            subDrivetrain));

    autoChooser.setDefaultOption("null", null);
    autoChooser.addOption("Four Ball A", autoFourBallA);
    SmartDashboard.putData(autoChooser);

    cargoState = CargoState.NONE;
  }

  private void configureButtonBindings() {

    // Driver Commands
    conDriver.btn_LBump
        .whenPressed(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedLow));
    conDriver.btn_LBump
        .whenReleased(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedMid));
    conDriver.btn_RBump
        .whenPressed(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedHigh));
    conDriver.btn_RBump
        .whenReleased(() -> subDrivetrain.setArcadeDriveSpeedMultiplier(prefDrivetrain.driveArcadeSpeedMid));

    // Operator Commands

    // Shooting
    conOperator.btn_RTrig.whileHeld(comShootCargo);
    conOperator.btn_RTrig.whileHeld(
        () -> subShooter.setMotorRPMToGoalRPM())
        .whenReleased(() -> subShooter.neutralOutput());

    conOperator.btn_RBump.whenPressed(() -> subShooter.setMotorRPMToGoalRPM());

    // Intake
    conOperator.btn_LTrig.whileHeld(comCollectCargo);
    conOperator.btn_Back.whenPressed(() -> subIntake.setRetracted());

    // Presets
    conOperator.POV_North.whenPressed(() -> subShooter.setGoalRPM(prefPreset.presetFenderShooterRPM));
    conOperator.POV_North.whenPressed(() -> subHood.setAngleDegrees(prefPreset.presetFenderHoodDegrees));

    conOperator.POV_South.whenPressed(() -> subShooter.setGoalRPM(prefPreset.presetLaunchpadShooterRPM));
    conOperator.POV_South.whenPressed(() -> subHood.setAngleDegrees(prefPreset.presetLaunchpadHoodDegrees));

    conOperator.POV_West.whenPressed(() -> subShooter.setGoalRPM(prefPreset.presetTarmacShooterRPM));
    conOperator.POV_West.whenPressed(() -> subHood.setAngleDegrees(prefPreset.presetTarmacHoodDegrees));

    // Switchboard Commands

    // btn_1 -> Send Values to SmartDashboard
    conSwitchboard.btn_1.whenPressed(() -> subDrivetrain.displayValuesOnDashboard());
    conSwitchboard.btn_1.whenPressed(() -> subHood.displayValuesOnDashboard());
    conSwitchboard.btn_1.whenPressed(() -> subIntake.displayValuesOnDashboard());
    conSwitchboard.btn_1.whenPressed(() -> subShooter.displayValuesOnDashboard());
    conSwitchboard.btn_1.whenPressed(() -> subTransfer.displayValuesOnDashboard());
    conSwitchboard.btn_1.whenReleased(() -> subDrivetrain.hideValuesOnDashboard());
    conSwitchboard.btn_1.whenReleased(() -> subHood.hideValuesOnDashboard());
    conSwitchboard.btn_1.whenReleased(() -> subIntake.hideValuesOnDashboard());
    conSwitchboard.btn_1.whenReleased(() -> subShooter.hideValuesOnDashboard());
    conSwitchboard.btn_1.whenReleased(() -> subTransfer.hideValuesOnDashboard());

    // btn_2 -> Use Hardcoded or Default Preference Values
    conSwitchboard.btn_2.whenPressed(() -> SN_Preferences.usePreferences());
    conSwitchboard.btn_2.whenReleased(() -> SN_Preferences.useDefaults());

  }

  public enum CargoState {
    SHOOTING, COLLECTING, DISGARDING, PROCESSING, NONE
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
