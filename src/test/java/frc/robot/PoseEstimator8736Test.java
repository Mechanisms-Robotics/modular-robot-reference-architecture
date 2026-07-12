package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

/**
 * Headless checks on the odometry math in PoseEstimator8736. Uses a local
 * kinematics object (NOT DriveConstants — that would drag CAN/vendor classes
 * into a unit test).
 */
class PoseEstimator8736Test {

    // Square drivebase, half-track 0.275 m — same shape as the real robot.
    private static final double HALF_TRACK = 0.275;

    private static SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(
            new Translation2d(HALF_TRACK, HALF_TRACK),   // FL
            new Translation2d(HALF_TRACK, -HALF_TRACK),  // FR
            new Translation2d(-HALF_TRACK, HALF_TRACK),  // BL
            new Translation2d(-HALF_TRACK, -HALF_TRACK)  // BR
        );
    }

    private static SwerveModulePosition[] straight(double meters) {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(meters, Rotation2d.kZero),
            new SwerveModulePosition(meters, Rotation2d.kZero),
            new SwerveModulePosition(meters, Rotation2d.kZero),
            new SwerveModulePosition(meters, Rotation2d.kZero),
        };
    }

    @Test
    void straightLineOdometryAdvancesX() {
        PoseEstimator8736 estimator = new PoseEstimator8736(
            kinematics(), Rotation2d.kZero, Pose2d.kZero);

        // All four wheels roll 1 m facing forward -> robot moved +1 m in X.
        estimator.updateOdometry(straight(1.0), null, 0.02);

        assertEquals(1.0, estimator.getEstimatedPose().getX(), 1e-6);
        assertEquals(0.0, estimator.getEstimatedPose().getY(), 1e-6);
        assertEquals(
            0.0, estimator.getEstimatedPose().getRotation().getRadians(), 1e-6);
    }

    @Test
    void gyrolessRotationIsIntegratedFromWheelDeltas() {
        PoseEstimator8736 estimator = new PoseEstimator8736(
            kinematics(), Rotation2d.kZero, Pose2d.kZero);

        // Spin in place by 0.1 rad: each wheel points tangent to its mounting
        // circle (radius r) and rolls r * dtheta.
        double r = Math.hypot(HALF_TRACK, HALF_TRACK);
        double dtheta = 0.1;
        double arc = r * dtheta;
        SwerveModulePosition[] positions = new SwerveModulePosition[] {
            // Tangent direction = module bearing + 90 degrees (CCW spin).
            new SwerveModulePosition(arc, Rotation2d.fromDegrees(45 + 90)),
            new SwerveModulePosition(arc, Rotation2d.fromDegrees(-45 + 90)),
            new SwerveModulePosition(arc, Rotation2d.fromDegrees(135 + 90)),
            new SwerveModulePosition(arc, Rotation2d.fromDegrees(-135 + 90)),
        };

        // gyroRotation = null forces the kinematics-integration fallback path.
        estimator.updateOdometry(positions, null, 0.02);

        assertEquals(
            dtheta,
            estimator.getEstimatedPose().getRotation().getRadians(),
            1e-6
        );
        // Spinning in place should not translate the robot.
        assertEquals(0.0, estimator.getEstimatedPose().getX(), 1e-6);
        assertEquals(0.0, estimator.getEstimatedPose().getY(), 1e-6);
    }

    @Test
    void resetPoseMovesBothEstimators() {
        PoseEstimator8736 estimator = new PoseEstimator8736(
            kinematics(), Rotation2d.kZero, Pose2d.kZero);

        Pose2d target = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(90));
        estimator.resetPose(target, straight(0.0));

        // The fused estimate AND the sim ground-truth twin must both move —
        // a past bug reset only one, desyncing simulated vision.
        assertEquals(3.0, estimator.getEstimatedPose().getX(), 1e-9);
        assertEquals(3.0, estimator.getSimulatedPose().getX(), 1e-9);
        assertEquals(2.0, estimator.getSimulatedPose().getY(), 1e-9);
    }
}
