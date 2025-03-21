package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  public PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
  public PhotonPoseEstimator photonPoseEstimator;
  public AprilTagFieldLayout atfl;

  public Vision() {
    PhotonCamera.setVersionCheckEnabled(false);
    photonCamera.setDriverMode(true);

    try {
      atfl = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    } catch (IOException e) {
      e.printStackTrace();
    }

    // Create pose estimator
    photonPoseEstimator =
        new PhotonPoseEstimator(
            atfl,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            photonCamera,
            Constants.VisionConstants.robotToCam);
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
