package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;
import frc.robot.CONSTANTS.FieldConstants;
import frc.robot.CONSTANTS.VisionConstants;

/**
 * The AprilTag vision subsystem. Owns any number of pose cameras (each behind
 * a {@link PoseCameraIO}) and forwards every accepted pose estimate into the
 * shared {@link PoseEstimator8736} with a timestamp, so the Kalman filter can
 * fuse it against odometry with latency compensation.
 *
 * <p>Estimates are gated here (not in the IOs) so every camera type gets the
 * same sanity checks. The pose estimator itself may additionally ignore
 * vision entirely while FollowPath runs.
 */
public class Vision extends SubsystemBase {
    private final PoseCameraIO[] ios;
    private final PoseCameraIOInputsAutoLogged[] inputs;

    private final PoseEstimator8736 poseEstimator;

    /** Cameras are logged by index in construction order ("Vision/0", "Vision/1", ...). */
    public Vision(PoseEstimator8736 poseEstimator, PoseCameraIO... ios) {
        this.ios = ios;
        this.poseEstimator = poseEstimator;
        this.inputs = new PoseCameraIOInputsAutoLogged[ios.length];

        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PoseCameraIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + i, inputs[i]);

            // Constantly feed vision measurements into the pose estimator
            for (int j = 0; j < inputs[i].timestampSeconds.length; j++) {
                if (!isPlausible(inputs[i], j)) {
                    continue;
                }

                int tagCount = Math.max(1, inputs[i].tagCounts[j]);
                double distance = inputs[i].avgTagDistancesMeters[j];

                // Scale trust with solution quality: noise grows roughly
                // with the square of tag distance and shrinks with more
                // tags in the solve. Smaller std dev = trust vision more.
                double scale = (distance * distance) / tagCount;
                double linearStdDev =
                    VisionConstants.LINEAR_STD_DEV_BASE * scale;
                double angularStdDev =
                    VisionConstants.ANGULAR_STD_DEV_BASE * scale;

                Logger.recordOutput(
                    "Vision/" + i + "/LinearStdDev", linearStdDev);

                this.poseEstimator.addVisionMeasurement(
                    inputs[i].poseEstimates[j].toPose2d(),
                    inputs[i].timestampSeconds[j],
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }
        }
    }

    /**
     * Physical-plausibility gates applied to every estimate before it can
     * touch the pose estimator. Rejects:
     * (a) solutions off the floor plane (|z| beyond threshold),
     * (b) solutions outside the field boundary (plus a bumper margin),
     * (c) single-tag solutions from far away (geometrically ambiguous).
     */
    private static boolean isPlausible(
            PoseCameraIOInputsAutoLogged input, int j) {
        var pose = input.poseEstimates[j];

        if (Math.abs(pose.getZ()) > VisionConstants.Z_THRESHOLD) {
            return false;
        }

        double margin = VisionConstants.FIELD_BORDER_MARGIN_METERS;
        if (pose.getX() < -margin
            || pose.getX() > FieldConstants.LENGTH + margin
            || pose.getY() < -margin
            || pose.getY() > FieldConstants.WIDTH + margin) {
            return false;
        }

        if (input.tagCounts[j] <= 1
            && input.avgTagDistancesMeters[j]
                > VisionConstants.MAX_SINGLE_TAG_DISTANCE_METERS) {
            return false;
        }

        return true;
    }
}
