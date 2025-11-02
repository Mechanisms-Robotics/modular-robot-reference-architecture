package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private PhotonPoseEstimator photonEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    // The AprilTag IDs that should ignored for pose estimation
    private ArrayList<Integer> ignoreIds = new ArrayList<>();

    public Vision(String cameraName, Transform3d cameraToRobot) {
        camera = new PhotonCamera(cameraName);

        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;
        
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        photonEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            cameraToRobot);

        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        for (PhotonPipelineResult result : results) {
            
            // remove any AprilTags that are in the "ignore" list
            // if the ignore list is empty, then this will be skipped
            for (PhotonTrackedTarget target : result.targets) {
                if (ignoreIds.contains(target.getFiducialId())) 
                    result.targets.remove(target);
            }

            visionEstimate = photonEstimator.update(result);        
            if (visionEstimate.isPresent()) {
                Pose3d poseEstimate = visionEstimate.get().estimatedPose;

                SmartDashboard.putNumber(cameraName + "/pose/x", poseEstimate.getX());
                SmartDashboard.putNumber(cameraName + "/pose/y", poseEstimate.getY());
                SmartDashboard.putNumber(cameraName + "/pose/heading", poseEstimate.getRotation().getAngle());

                // TODO: send to pose estimator here
            }
        }
    }
}
