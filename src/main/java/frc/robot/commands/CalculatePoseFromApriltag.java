// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CalculatePoseFromApriltag extends CommandBase {
  Vision subVision;
  Drivetrain subDrivetrain;

  public CalculatePoseFromApriltag(Vision subVision, Drivetrain subDrivetrain) {
    this.subVision = subVision;
    this.subDrivetrain = subDrivetrain;

    addRequirements(subVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // PhotonVision will add something similar to this in their documentation!
    // https://docs.photonvision.org/en/latest/docs/examples/apriltag.html

    // Notable PhotonVision updates
    /*
     * https://docs.photonvision.org/en/latest/docs/integration/aprilTagStrategies.
     * html
     *
     * https://github.com/PhotonVision/photonvision/pull/571
     * - The getFieldToRobot thing suggested might actually just... do this
     */

    // Notable WPI updates
    /*
     * https://github.com/wpilibsuite/allwpilib/pull/4421
     * - included in the WPI 2023 Beta 3 release
     *
     * https://github.com/wpilibsuite/allwpilib/pull/4578
     */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (subVision.limelight.getLatestResult().hasTargets()) {
      // camera Position relative to robot @ 0,0
      // TODO: GET THESE VALUES & move to constants eventually
      double cameraXPosition = 0;
      double cameraYPosition = 0;
      double cameraZPosition = 0;
      double cameraPitch = 0;
      double cameraYaw = 0;
      double cameraRoll = 0;

      Pose3d cameraPose = new Pose3d(cameraXPosition, cameraYPosition, cameraZPosition,
          new Rotation3d(cameraRoll, cameraPitch, cameraYaw));
      Transform3d cameraToRobotTrans = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), cameraPose);

      PhotonPipelineResult result = subVision.limelight.getLatestResult();

      // 1. you know where your camera is relative to the target
      Transform3d camToTargetTrans = result.getBestTarget().getBestCameraToTarget();

      // 2. you know where the target is relative to the field
      // currently implementing w/ 1 point @ origin
      // this is the part WPI will handle IN 2023
      Pose3d aprilTagPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

      // 3. determine where your camera is relative to field
      Pose3d cameraOnFieldPose = aprilTagPose.transformBy(camToTargetTrans);

      // 4. you know where your camera is relative to you
      Pose3d robotOnFieldPose = cameraOnFieldPose.transformBy(cameraToRobotTrans);

      SmartDashboard.putNumber("robotOnFieldPose Z", robotOnFieldPose.getZ());

      subDrivetrain.resetPose(robotOnFieldPose.toPose2d());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
