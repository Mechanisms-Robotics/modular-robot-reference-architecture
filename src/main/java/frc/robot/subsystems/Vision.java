package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final String cameraName;
    private final Transform3d cameraToRobot;

    private PhotonPoseEstimator poseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    public Vision(String cameraName, Transform3d cameraToRobot) {

        this.cameraName = cameraName;
        this.cameraToRobot = cameraToRobot;

        camera = new PhotonCamera(cameraName);

        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        poseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, 
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            cameraToRobot);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            // TODO: manage apriltags here
        }
    }
}
