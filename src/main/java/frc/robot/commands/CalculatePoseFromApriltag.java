// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotPreferences.prefVision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class CalculatePoseFromApriltag extends CommandBase {
  Vision subVision;
  Drivetrain subDrivetrain;

  PhotonPipelineResult result;
  Optional<PhotonTrackedTarget> filteredResult;
  PhotonTrackedTarget target;
  PhotonTrackedTarget lastTarget;

  public CalculatePoseFromApriltag(Vision subVision, Drivetrain subDrivetrain) {
    this.subVision = subVision;
    this.subDrivetrain = subDrivetrain;

    addRequirements(subVision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    // Calculates a field-relative robot pose using 1 constant april tag location.

    // PhotonVision will add something similar to this in their documentation!
    // https://docs.photonvision.org/en/latest/docs/examples/apriltag.html

    // Notable PhotonVision updates
    /*
     * https://docs.photonvision.org/en/latest/docs/integration/aprilTagStrategies.
     * html
     *
     * https://github.com/PhotonVision/photonvision/pull/571
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
    result = subVision.limelight.getLatestResult();

    // Filter it down to just 1 tag position that we know is accurate
    // t.equals(lastTarget) is used to check if the camera has updated
    // convert to !t.equals(lastTargetMap[t.getFiducialId()]) when we need mutliple
    // tag locations
    filteredResult = result.getTargets().stream()
        .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
        .findFirst();

    // if the filtered result exists, we see a target
    if (filteredResult.isPresent()) {
      target = filteredResult.get();
      lastTarget = target;

      // camera Position relative to robot @ 0,0
      Pose3d cameraPose = new Pose3d(prefVision.cameraXPosition.getValue(), prefVision.cameraYPosition.getValue(),
          prefVision.cameraZPosition.getValue(),
          new Rotation3d(prefVision.cameraRoll.getValue(), prefVision.cameraPitch.getValue(),
              prefVision.cameraYaw.getValue()));

      Transform3d cameraToRobotTrans = new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), cameraPose);

      // 1. you know where your camera is relative to the target
      Transform3d camToTargetTrans = result.getBestTarget().getBestCameraToTarget();

      // 2. you know where the target is relative to the field
      // currently implementing w/ 1 point @ origin, WPI will handle in 2023
      Pose3d aprilTagPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

      // 3. determine where your camera is relative to field
      Pose3d cameraOnFieldPose = aprilTagPose.transformBy(camToTargetTrans);

      // 4. you know where your camera is relative to you
      Pose3d robotOnFieldPose = cameraOnFieldPose.transformBy(cameraToRobotTrans);

      // Reset the robot's 2d pose
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
