package frc.robot.commands;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.FieldConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.FieldUtil;

/**
 * Follows a pre-planned Choreo trajectory with a holonomic feedback
 * controller: each loop we sample the trajectory at the elapsed time, compare
 * the sampled pose against our estimated pose, and command chassis speeds
 * that chase the sample.
 *
 * <p>Two independent kinds of flipping can apply to a path:
 * <ul>
 *   <li><b>Alliance mirroring</b> — handled by Choreo itself via
 *       {@code sampleAt(t, isRedAlliance)}.</li>
 *   <li><b>{@code isMirrored}</b> — OUR left/right reflection across the
 *       field's long axis, for reusing one path on the other side of the
 *       field for the same alliance.</li>
 * </ul>
 *
 * <p>Known limitation (see TODO below): HolonomicDriveController assumes the
 * robot travels in the direction its pose faces, which is not generally true
 * for a swerve path that strafes — Choreo's per-sample vx/vy feedforwards are
 * collapsed to a scalar speed. Paths whose heading tracks the direction of
 * travel work fine; heavy-strafe paths will track loosely.
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
    private final HolonomicDriveController holonomicController;

    public FollowPath(
            Trajectory<SwerveSample> trajectory,
            Drivetrain drivetrain,
            boolean resetPose,
            boolean isMirrored) {

        this.trajectory = trajectory;
        this.drivetrain = drivetrain;
        this.resetPose = resetPose;
        this.isMirrored = isMirrored;

        // Constraints are (maxVelocity, maxAcceleration) in rad/s and rad/s^2.
        // This previously passed ANGLE_MAX_ACCELERATION for both, letting the
        // profile demand 20 rad/s of rotation (2.5x our configured max).
        Constraints thetaProfile = new TrapezoidProfile.Constraints(
            CONSTANTS.DriveConstants.ANGLE_MAX_VELOCITY,
            CONSTANTS.DriveConstants.ANGLE_MAX_ACCELERATION);

        ProfiledPIDController thetaController = new ProfiledPIDController(
            CONSTANTS.PATH_FOLLOWER_P_THETA, 0, 0, thetaProfile);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        holonomicController = new HolonomicDriveController(
            new PIDController(CONSTANTS.PATH_FOLLOWER_P_X, 0, 0),
            new PIDController(CONSTANTS.PATH_FOLLOWER_P_Y, 0, 0),
            thetaController
        );

        super.addRequirements(drivetrain);
    }

    public FollowPath(
            Trajectory<SwerveSample> trajectory,
            Drivetrain drivetrain,
            boolean resetPose) {
        this(trajectory, drivetrain, resetPose, false);
    }

    // -----------------
    // COMMAND LIFECYCLE
    // -----------------

    @Override
    public void initialize() {
        // The trajectory is indexed by time-since-start; the timer is that clock.
        timer.reset();
        timer.start();

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
                // TODO: Why would this ever happen? Should we handle it differently?
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
            return; // TODO: Why would this ever happen? Should we handle it differently?
        }
        SwerveSample sample;

        if (this.isMirrored) {
            // Reflect the sample across the field's long (X) axis: Y and
            // heading flip sign, so every Y-component and every angular
            // quantity (heading, omega, AND alpha) must be negated. The
            // reflection also turns left-side modules into right-side ones,
            // so the per-module force arrays swap FL<->FR and BL<->BR
            // (Choreo module order: FL, FR, BL, BR).
            SwerveSample original = swerveSample.get();
            double[] forcesX = original.moduleForcesX();
            double[] forcesY = original.moduleForcesY();
            sample = new SwerveSample(
                original.t,
                original.x,
                FieldConstants.WIDTH - original.y,
                -original.heading,
                original.vx,
                -original.vy,
                -original.omega,
                original.ax,
                -original.ay,
                -original.alpha,
                new double[] {
                    forcesX[1], forcesX[0], forcesX[3], forcesX[2]
                },
                new double[] {
                    -forcesY[1], -forcesY[0], -forcesY[3], -forcesY[2]
                }
            );
        } else {
            sample = swerveSample.get();
        }

        // TODO: This loses the capability of Choreo to control the wheels optimally. See the choreo docs.

        // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html

        ChassisSpeeds sampleSpeeds = sample.getChassisSpeeds();

        double desiredLinearVelocity = Math.sqrt(
            sampleSpeeds.vxMetersPerSecond * sampleSpeeds.vxMetersPerSecond +
            sampleSpeeds.vyMetersPerSecond * sampleSpeeds.vyMetersPerSecond);

        ChassisSpeeds commandedSpeeds = this.holonomicController.calculate(
            this.drivetrain.getPose(),
            sample.getPose(),
            desiredLinearVelocity,
            sample.getPose().getRotation()
        );

        this.drivetrain.setDesiredState(commandedSpeeds);
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
        // TODO: If precition is important we may need an end-state controller.
        return this.timer.get() >= this.trajectory.getTotalTime(); // TODO: I assume total time is in seconds?
    }
}
