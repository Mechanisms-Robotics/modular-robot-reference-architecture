package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.PoseEstimator8736;
import frc.robot.CONSTANTS.FieldConstants;

/**
 * PoseCameraIO for simulation: renders the AprilTag layout through a
 * PhotonVision camera sim (with realistic noise, FOV, and latency) and runs
 * the same pose-estimation pipeline as the real camera IO.
 *
 * <p>The sim needs to know where the robot "actually" is to know which tags
 * are visible — that ground truth comes from the odometry-only estimator in
 * {@link PoseEstimator8736#getSimulatedPose()}, NOT the fused estimate (using
 * the fused estimate would let vision feed itself its own corrections).
 */
public class PoseCameraIOSim implements PoseCameraIO {
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    private final PoseEstimator8736 poseEstimator;

    private final PhotonCamera camera;
    private final String cameraName;

    // Transform FROM the robot center TO the camera lens (PhotonVision's
    // "robotToCamera" convention) — see PoseCameraIOPhoton.
    private final Transform3d robotToCamera;

    private final PhotonPoseEstimator photonEstimator;

    // PoseEstimator is passed in because the sim camera needs the robot's current position to update.
    public PoseCameraIOSim(String cameraName, Transform3d robotToCamera, PoseEstimator8736 poseEstimator) {
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;

        this.visionSim = new VisionSystemSim("visionSim");
        this.visionSim.addAprilTags(FieldConstants.APRILTAG_FIELD_LAYOUT);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.17, 0.05);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(60);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        this.camera = new PhotonCamera(cameraName);
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);

        this.cameraSim.enableDrawWireframe(true);

        this.visionSim.addCamera(cameraSim, robotToCamera);

        this.photonEstimator = new PhotonPoseEstimator(
            FieldConstants.APRILTAG_FIELD_LAYOUT,
            this.robotToCamera);

        this.poseEstimator = poseEstimator;
    }

    @Override
    public void updateInputs(PoseCameraIOInputs inputs) {
        inputs.isConnected = true;

        List<PhotonPipelineResult> results = this.camera.getAllUnreadResults();
        Optional<EstimatedRobotPose> visionEstimate = Optional.empty();

        List<Double> timestampSecondsArray = new ArrayList<>();
        List<Pose3d> poseEstimatesArray = new ArrayList<>();
        List<Integer> tagCountsArray = new ArrayList<>();
        List<Double> avgTagDistancesArray = new ArrayList<>();

        for (PhotonPipelineResult result : results) {
            visionEstimate = this.photonEstimator.estimateCoprocMultiTagPose(result);

            if (visionEstimate.isEmpty()) {
                visionEstimate = this.photonEstimator.estimateLowestAmbiguityPose(result);
            }

            if (visionEstimate.isPresent()) {
                Pose3d poseEstimate = visionEstimate.get().estimatedPose;

                Logger.recordOutput(cameraName + "/simPose", poseEstimate);

                // Push each unread input to the arrays
                timestampSecondsArray.add(visionEstimate.get().timestampSeconds);
                poseEstimatesArray.add(poseEstimate);
                tagCountsArray.add(visionEstimate.get().targetsUsed.size());
                avgTagDistancesArray.add(
                    PoseCameraIOPhoton.averageTagDistance(visionEstimate.get()));
            }
        }

        // Finally, push all estimates to the inputs
        inputs.timestampSeconds = timestampSecondsArray.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.poseEstimates = poseEstimatesArray.stream().toArray(Pose3d[]::new);
        inputs.tagCounts = tagCountsArray.stream().mapToInt(Integer::intValue).toArray();
        inputs.avgTagDistancesMeters = avgTagDistancesArray.stream().mapToDouble(Double::doubleValue).toArray();

        // Advance the simulated camera to the robot's current ground-truth
        // pose. Done AFTER reading results, so frames produced here are read
        // next loop — one loop of extra latency, comparable to a real camera.
        visionSim.update(poseEstimator.getSimulatedPose());
    }
}