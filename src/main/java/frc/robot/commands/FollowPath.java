package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.FieldUtil;
import org.littletonrobotics.junction.Logger;

/**
 * Follows a pre-planned Choreo trajectory.
 *
 * <p>Control law (the follower Choreo's docs recommend): take the sample's
 * field-relative velocities (vx, vy, omega) as feedforward — they already
 * encode everything the path optimizer knows about how the robot should move,
 * including strafing — and add a proportional position correction per axis:
 *
 * <pre>
 *   vx     = sample.vx    + kP * (sample.x - pose.x)
 *   vy     = sample.vy    + kP * (sample.y - pose.y)
 *   omega  = sample.omega + kP * wrap(sample.heading - pose.heading)
 * </pre>
 *
 * This replaces the previous WPILib HolonomicDriveController approach, which
 * collapsed (vx, vy) into a scalar speed pointed along the sample's heading —
 * correct only when the robot travels nose-first, which swerve paths rarely do.
 *
 * <p>Two independent kinds of flipping can apply to a path:
 * <ul>
 *   <li><b>Alliance mirroring</b> — handled by Choreo itself via
 *       {@code sampleAt(t, isRedAlliance)}.</li>
 *   <li><b>{@code isMirrored}</b> — OUR left/right reflection across the
 *       field's long axis (see {@link #mirrorSample}), for reusing one path on
 *       the other side of the field for the same alliance.</li>
 * </ul>
 */
public class FollowPath extends Command {

    private final Trajectory<SwerveSample> trajectory;
    private final Drivetrain drivetrain;

    // Whether to teleport the pose estimate to the path's start point when
    // the command starts. Use for the FIRST path of auto only; mid-sequence
    // paths must keep the estimator's continuity.
    private final boolean resetPose;

    private boolean isRedAlliance;
    private final boolean isMirrored;

    private final Timer timer = new Timer();

    // Per-axis position feedback. Gains are 1/s: output velocity per meter
    // (or radian) of error, added on top of the trajectory feedforward.
    private final PIDController xController = new PIDController(
        CONSTANTS.PATH_FOLLOWER_P_X, 0, 0);
    private final PIDController yController = new PIDController(
        CONSTANTS.PATH_FOLLOWER_P_Y, 0, 0);
    private final PIDController headingController = new PIDController(
        CONSTANTS.PATH_FOLLOWER_P_THETA, 0, 0);

    public FollowPath(
            Trajectory<SwerveSample> trajectory,
            Drivetrain drivetrain,
            boolean resetPose,
            boolean isMirrored) {

        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        this.resetPose = resetPose;
        this.isMirrored = isMirrored;

        // Heading error must wrap so a 350 -> 10 degree correction goes 20
        // degrees the short way, not 340 degrees the long way.
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);

        super.addRequirements(drivetrain);
    }

    public FollowPath(
            Trajectory<SwerveSample> trajectory,
            Drivetrain drivetrain,
            boolean resetPose) {
        this(trajectory, drivetrain, resetPose, false);
    }

    /**
     * Reflects a sample across the field's long (X) axis: every Y quantity
     * and every angular quantity (heading, omega, alpha) negates, and the
     * reflection turns left-side modules into right-side ones, so the
     * per-module force arrays swap FL&lt;-&gt;FR and BL&lt;-&gt;BR (Choreo
     * module order: FL, FR, BL, BR).
     *
     * <p>Static and package-visible so it is unit-testable.
     */
    static SwerveSample mirrorSample(SwerveSample sample, double fieldWidth) {
        double[] forcesX = sample.moduleForcesX();
        double[] forcesY = sample.moduleForcesY();
        return new SwerveSample(
            sample.t,
            sample.x,
            fieldWidth - sample.y,
            -sample.heading,
            sample.vx,
            -sample.vy,
            -sample.omega,
            sample.ax,
            -sample.ay,
            -sample.alpha,
            new double[] {
                forcesX[1], forcesX[0], forcesX[3], forcesX[2]
            },
            new double[] {
                -forcesY[1], -forcesY[0], -forcesY[3], -forcesY[2]
            }
        );
    }

    // -----------------
    // COMMAND LIFECYCLE
    // -----------------

    @Override
    public void initialize() {
        // The trajectory is indexed by time-since-start; the timer is that clock.
        timer.reset();
        timer.start();

        this.xController.reset();
        this.yController.reset();
        this.headingController.reset();

        // disable vision updates while following a path
        this.drivetrain.poseEstimator.setVisionEnabled(false);

        // Sampled once at path start: Choreo mirrors the trajectory for red, and
        // the alliance cannot change mid-path.
        this.isRedAlliance = FieldUtil.isRedAlliance();

        if (this.resetPose) {
            // rotate the initial pose if we're on the red alliance
            Optional<Pose2d> initialPose = trajectory.getInitialPose(
                this.isRedAlliance);

            if (initialPose.isEmpty()) {
                // Only possible with an empty/corrupted trajectory file;
                // better to fail loudly at the start of auto than drive from
                // a wrong origin.
                throw new IllegalStateException("Trajectory has no initial pose!");
            }
            this.drivetrain.resetPose(
                this.isMirrored
                    ? FieldUtil.flipPose(initialPose.get())
                    : initialPose.get());
        }
    }

    @Override
    public void execute() {
        double t = this.timer.get();

        Optional<SwerveSample> swerveSample = this.trajectory.sampleAt(
            t, isRedAlliance);
        if (swerveSample.isEmpty()) {
            // Can only happen for an empty trajectory; isFinished() will end
            // us at totalTime = 0 immediately.
            return;
        }

        SwerveSample sample = this.isMirrored
            ? mirrorSample(swerveSample.get(), CONSTANTS.FieldConstants.WIDTH)
            : swerveSample.get();

        Pose2d pose = this.drivetrain.getPose();

        // Feedforward from the plan + proportional feedback on position
        // error, all in FIELD-relative terms.
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(
            sample.vx + this.xController.calculate(pose.getX(), sample.x),
            sample.vy + this.yController.calculate(pose.getY(), sample.y),
            sample.omega +
                this.headingController.calculate(
                    pose.getRotation().getRadians(),
                    sample.heading
                )
        );

        Logger.recordOutput("FollowPath/SamplePose", sample.getPose());
        Logger.recordOutput("FollowPath/FieldSpeeds", fieldRelativeSpeeds);

        // Convert to the robot frame using our ACTUAL heading. Note: this is
        // the raw pose rotation, not DrivetrainController's driver-relative
        // version — paths live in field coordinates, not driver coordinates.
        ChassisSpeeds robotRelativeSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds,
                pose.getRotation()
            );

        this.drivetrain.setDesiredState(robotRelativeSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        // Re-enable vision fusion. initialize() turned it off for the
        // duration of the path; without this line the robot would run the
        // entire rest of the match blind to AprilTags.
        this.drivetrain.poseEstimator.setVisionEnabled(true);

        this.drivetrain.setDesiredState(new ChassisSpeeds()); // TODO: There may be cases where we don't want the robot to stop!
        this.timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Time-based only: we declare done when the plan's clock runs out,
        // wherever we are. TODO: add a position tolerance / end-state
        // controller if precision matters for scoring.
        return this.timer.get() >= this.trajectory.getTotalTime();
    }
}
