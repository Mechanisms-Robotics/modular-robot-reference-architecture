package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;
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
                // Sanity gate: the robot drives on the floor, so a solution
                // that puts it half a meter above OR below the carpet is a
                // bad tag solve. The old check only rejected above-floor
                // poses; Math.abs also catches below-floor ones.
                double z = inputs[i].poseEstimates[j].getZ();
                if (Math.abs(z) > VisionConstants.Z_THRESHOLD) {
                    continue;
                }

                // Fixed measurement std devs (x meters, y meters, theta rad):
                // large-ish values = "trust vision loosely", letting odometry
                // dominate short-term motion while vision slowly corrects
                // drift. TODO: scale with tag distance/count instead.
                this.poseEstimator.addVisionMeasurement(
                    inputs[i].poseEstimates[j].toPose2d(),
                    inputs[i].timestampSeconds[j],
                    VecBuilder.fill(0.9, 0.9, 0.9)
                );
            }
        }
    }
}
