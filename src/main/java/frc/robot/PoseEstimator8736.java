package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Utility class for managing swerve drive pose estimation. Encapsulates the SwerveDrivePoseEstimator
 * and handles odometry updates, vision measurements, and gyro integration.
 *
 * <p>Two estimators run side by side:
 * <ul>
 *   <li>{@code poseEstimator} — the real estimate: odometry + gyro fused with
 *       (optionally gated) vision measurements. This is what the robot acts on.</li>
 *   <li>{@code simulatedPoseEstimator} — identical odometry input but NO
 *       vision. In simulation it serves as ground truth for rendering the
 *       simulated cameras (vision can't be allowed to feed itself). On a real
 *       robot it's a cheap odometry-only reference trace in the logs.</li>
 * </ul>
 */
public class PoseEstimator8736 {

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    // Odometry-only twin of poseEstimator; see class javadoc.
    private final SwerveDrivePoseEstimator simulatedPoseEstimator;

    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
        };

    // Vision fusion is on by default; FollowPath turns it off during path
    // following so a bad tag sighting can't yank the pose mid-trajectory.
    private boolean visionEnabled = true;

    /**
     * Creates a new PoseEstimator.
     *
     * @param kinematics The swerve drive kinematics
     * @param initialGyroRotation The initial gyro rotation
     * @param initialPose The initial pose estimate
     */
    public PoseEstimator8736(
        SwerveDriveKinematics kinematics,
        Rotation2d initialGyroRotation,
        Pose2d initialPose
    ) {
        this.kinematics = kinematics;
        this.rawGyroRotation = initialGyroRotation;
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            initialPose
        );

        this.simulatedPoseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            rawGyroRotation,
            lastModulePositions,
            initialPose
        );
    }

    /**
     * Updates the pose estimator with odometry data.
     *
     * @param modulePositions The current module positions
     * @param gyroRotation The current gyro rotation (or null to use kinematics)
     * @param timestamp The timestamp of this sample
     */
    public void updateOdometry(
        SwerveModulePosition[] modulePositions,
        Rotation2d gyroRotation,
        double timestamp
    ) {
        // Calculate module deltas
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] = new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters -
                    lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle
            );
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroRotation != null) {
            // Use the real gyro angle
            rawGyroRotation = gyroRotation;
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(
                new Rotation2d(twist.dtheta)
            );
        }

        // Apply update to pose estimator
        poseEstimator.updateWithTime(
            timestamp,
            rawGyroRotation,
            modulePositions
        );

        this.simulatedPoseEstimator.updateWithTime(
            timestamp,
            rawGyroRotation,
            modulePositions
        );
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionRobotPoseMeters The vision-measured robot pose
     * @param timestampSeconds The timestamp of the vision measurement
     * @param visionMeasurementStdDevs The standard deviations of the vision measurement
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        if (!visionEnabled) {
            return; // ignore vision measurements if vision is disabled
        }

        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds,
            visionMeasurementStdDevs
        );
    }

    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionRobotPoseMeters The vision-measured robot pose
     * @param timestampSeconds The timestamp of the vision measurement
     */
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds
    ) {
        if (!visionEnabled) {
            return; // ignore vision measurements if vision is disabled
        }

        poseEstimator.addVisionMeasurement(
            visionRobotPoseMeters,
            timestampSeconds
        );
    }

    /**
     * Resets the pose estimator to a specific pose.
     *
     * @param pose The new pose
     * @param modulePositions The current module positions
     */
    public void resetPose(Pose2d pose, SwerveModulePosition[] modulePositions) {
        poseEstimator.resetPosition(rawGyroRotation, modulePositions, pose);

        // Keep the sim ground-truth estimator in step. Without this, a pose
        // reset (start of auto, driver re-zero) moved the "real" estimate but
        // not the simulated ground truth, so the sim vision system rendered
        // tags from a stale robot position from then on.
        simulatedPoseEstimator.resetPosition(
            rawGyroRotation,
            modulePositions,
            pose
        );
    }

    /**
     * Returns the current estimated pose.
     *
     * @return The current pose estimate
     */
    @AutoLogOutput(key = "PoseEstimator8736/EstimatedPosition")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Only use this in simulation to get the actual simulated position of the robot.
     * This position is determined by odometry.
     * 
     * @return actual simulated position
     */
    @AutoLogOutput(key = "Simulation/ActualPosition")
    public Pose2d getSimulatedPose() {
        return this.simulatedPoseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current raw gyro rotation.
     *
     * @return The raw gyro rotation
     */
    public Rotation2d getRawGyroRotation() {
        return rawGyroRotation;
    }
}