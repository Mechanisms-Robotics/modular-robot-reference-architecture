package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the IMU. Implementations exist for the Redux
 * Canandgyro ({@link GyroIORedux}) and CTRE Pigeon 2 ({@link GyroIOCTRE});
 * simulation uses {@code new GyroIO() {}} — the no-op defaults report
 * "disconnected", which makes the pose estimator fall back to wheel-delta
 * (kinematics) heading.
 */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {

        /** False when the device is missing/unresponsive; heading falls back to kinematics. */
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;

        // High-frequency yaw samples captured by PhoenixOdometryThread since
        // the last loop. Index-aligned with each module's odometry samples so
        // sample i across all devices shares timestamp i.
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    /** Refreshes all fields of {@code inputs} from the hardware. */
    public default void updateInputs(GyroIOInputs inputs) {}

    /** Re-zeros the physical gyro's yaw. Pose-level heading resets usually go through PoseEstimator8736 instead. */
    public default void zeroGyro() {}
}
