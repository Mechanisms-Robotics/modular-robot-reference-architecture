package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.CONSTANTS.FieldConstants;
import org.junit.jupiter.api.Test;

/**
 * Pure-math checks on the field mirroring used by mirrored autos. These run
 * headless (no HAL) — FieldConstants only loads the AprilTag layout JSON.
 */
class FieldUtilTest {

    private static final double EPS = 1e-9;

    @Test
    void flipPoseReflectsYAcrossFieldCenterline() {
        Pose2d pose = new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(30));
        Pose2d flipped = FieldUtil.flipPose(pose);

        // X is unchanged; Y reflects across the field width; heading negates.
        assertEquals(3.0, flipped.getX(), EPS);
        assertEquals(FieldConstants.WIDTH - 1.0, flipped.getY(), EPS);
        assertEquals(-30.0, flipped.getRotation().getDegrees(), EPS);
    }

    @Test
    void flipPoseTwiceIsIdentity() {
        Pose2d pose = new Pose2d(5.5, 2.25, Rotation2d.fromDegrees(-135));
        Pose2d roundTrip = FieldUtil.flipPose(FieldUtil.flipPose(pose));

        assertEquals(pose.getX(), roundTrip.getX(), EPS);
        assertEquals(pose.getY(), roundTrip.getY(), EPS);
        assertEquals(
            pose.getRotation().getRadians(),
            roundTrip.getRotation().getRadians(),
            EPS
        );
    }

    @Test
    void flipPoseKeepsCenterlinePointsOnCenterline() {
        double centerY = FieldConstants.WIDTH / 2.0;
        Pose2d pose = new Pose2d(7.0, centerY, Rotation2d.kZero);
        Pose2d flipped = FieldUtil.flipPose(pose);

        assertEquals(centerY, flipped.getY(), EPS);
        assertEquals(0.0, flipped.getRotation().getRadians(), EPS);
    }
}
