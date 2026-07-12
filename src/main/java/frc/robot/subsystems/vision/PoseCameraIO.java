package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Hardware abstraction for one pose-estimating camera. Implementations solve
 * tag poses however they like (real PhotonVision coprocessor, simulated
 * camera, ...); the subsystem only ever sees field-relative robot poses plus
 * capture timestamps.
 */
public interface PoseCameraIO {
    @AutoLog
    public static class PoseCameraIOInputs {
        public boolean isConnected = false;

        // One entry per NEW pose estimate produced since the last loop
        // (may be empty). Arrays are index-aligned: poseEstimates[k] was
        // captured at timestampSeconds[k] (FPGA epoch), which is what the
        // pose estimator needs for latency compensation. tagCounts[k] and
        // avgTagDistancesMeters[k] describe the same estimate so the
        // subsystem can decide how much to trust it.
        public double[] timestampSeconds = new double[] {};
        public Pose3d[] poseEstimates = new Pose3d[] {};
        public int[] tagCounts = new int[] {};
        public double[] avgTagDistancesMeters = new double[] {};
    }

    /** Refreshes all fields of {@code inputs} from the camera. */
    public default void updateInputs(PoseCameraIOInputs inputs) {}
}