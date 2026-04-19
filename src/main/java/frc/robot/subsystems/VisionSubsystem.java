package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class VisionSubsystem {

    private final PhotonCamera camera;

    private final Transform3d robotToCam = new Transform3d(
        new Translation3d(
            Constants.VisionConstants.CAMERA_FORWARD_METERS,
            Constants.VisionConstants.CAMERA_SIDE_METERS,
            Constants.VisionConstants.CAMERA_HEIGHT_METERS
        ),
        new Rotation3d(
            Constants.VisionConstants.CAMERA_ROLL_RADIANS,
            Constants.VisionConstants.CAMERA_PITCH_RADIANS,
            Constants.VisionConstants.CAMERA_YAW_RADIANS
        )
    );

    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator poseEstimator;

    public VisionSubsystem() {
        camera = new PhotonCamera(Constants.VisionConstants.CAMERA_NAME);

        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCam
        );

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public boolean hasTargets() {
        return getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget();
        }
        return null;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        return poseEstimator.update(result);
    }

    public void updateDashboard() {
        PhotonPipelineResult result = getLatestResult();

        SmartDashboard.putBoolean("Vision/Has Targets", result.hasTargets());

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            SmartDashboard.putNumber("Vision/Best Target ID", target.getFiducialId());
            SmartDashboard.putNumber("Vision/Yaw", target.getYaw());
            SmartDashboard.putNumber("Vision/Pitch", target.getPitch());
            SmartDashboard.putNumber("Vision/Area", target.getArea());

            var camToTarget = target.getBestCameraToTarget();
            SmartDashboard.putNumber("Vision/Target X", camToTarget.getX());
            SmartDashboard.putNumber("Vision/Target Y", camToTarget.getY());
            SmartDashboard.putNumber("Vision/Target Z", camToTarget.getZ());
        } else {
            SmartDashboard.putNumber("Vision/Best Target ID", -1);
            SmartDashboard.putNumber("Vision/Yaw", 0.0);
            SmartDashboard.putNumber("Vision/Pitch", 0.0);
            SmartDashboard.putNumber("Vision/Area", 0.0);
            SmartDashboard.putNumber("Vision/Target X", 0.0);
            SmartDashboard.putNumber("Vision/Target Y", 0.0);
            SmartDashboard.putNumber("Vision/Target Z", 0.0);
        }

        Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();

        if (estimatedPose.isPresent()) {
            Pose3d pose = estimatedPose.get().estimatedPose;

            SmartDashboard.putBoolean("Vision/Pose Present", true);
            SmartDashboard.putNumber("Vision/Robot Pose X", pose.getX());
            SmartDashboard.putNumber("Vision/Robot Pose Y", pose.getY());
            SmartDashboard.putNumber("Vision/Robot Pose Z", pose.getZ());
            SmartDashboard.putNumber("Vision/Robot Rotation Deg",
                pose.getRotation().toRotation2d().getDegrees());
        } else {
            SmartDashboard.putBoolean("Vision/Pose Present", false);
        }
    }
}