// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frcteam3255.joystick.SN_F310Gamepad;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotMap.mapControllers;
import frc.robot.RobotPreferences.prefDrivetrain;
import frc.robot.commands.FourBallA;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  // Controllers
  private final SN_F310Gamepad conDriver = new SN_F310Gamepad(mapControllers.DRIVER_CONTROLLER);
  private final SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(
      prefDrivetrain.driveSlewRateLimit.getValue());

  private final SN_F310Gamepad conOperator = new SN_F310Gamepad(mapControllers.OPERATOR_CONTROLLER);

  // Subsystems
  private final Drivetrain subDrivetrain = new Drivetrain();

  // Autos
  private final FourBallA autoFourBallA = new FourBallA(subDrivetrain);
  SendableChooser<Command> autoChooser = new SendableChooser<>();

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

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
