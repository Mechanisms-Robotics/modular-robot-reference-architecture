package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.CONSTANTS.FieldConstants;

/**
 * Small stateless helpers for reasoning about the field and our alliance.
 *
 * <p>All pose math in this codebase uses the WPILib blue-alliance-origin
 * convention (+X toward the red alliance wall, +Y left, CCW positive), so any
 * alliance-dependent behavior should funnel through this class instead of
 * sprinkling {@code DriverStation.getAlliance()} checks around the codebase.
 */
public class FieldUtil {

    private FieldUtil() {} // static utility class; never instantiate

    /**
     * Returns our alliance, or Blue if the Driver Station has not told us yet
     * (before connecting to the FMS/DS the alliance is unknown). Blue is the
     * safe default because field coordinates are blue-origin: treating an
     * unknown alliance as Blue means "apply no flipping".
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    /**
     * Returns true only when the Driver Station has affirmatively told us we
     * are on the red alliance. Unknown alliance counts as "not red" so that
     * all red-flipping logic defaults to doing nothing.
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .map(alliance -> alliance == Alliance.Red)
            .orElse(false);
    }

    /**
     * Mirrors a pose across the field's long (X) axis: same X, reflected Y,
     * negated heading. This is for running a path authored on one side of the
     * field on the other side for the same alliance (left/right symmetry).
     * It is NOT the blue-to-red alliance flip — Choreo handles alliance
     * mirroring itself via {@code sampleAt(t, mirrorForRed)}.
     */
    public static Pose2d flipPose(Pose2d pose) {
        return new Pose2d(
            pose.getX(),
            FieldConstants.WIDTH - pose.getY(),
            pose.getRotation().unaryMinus()
        );
    }
}
