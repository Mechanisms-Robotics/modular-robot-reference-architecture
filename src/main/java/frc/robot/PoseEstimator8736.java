package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/*
 * TEST PLAN:
 *
 * Determine where to initialize the pose estimator.
 * Test output of getModulePositions to make sure they are correct.
 */

public class PoseEstimator8736 {

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs =
        new GyroIOInputsAutoLogged();

    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Drivetrain drivetrain;

    public PoseEstimator8736(GyroIO gyroIO) {
        this.gyroIO = gyroIO;
    }

    // TODO: Where do we call this from and do we need to make sure drivetrain encoders are set first?
    // Can we just call this whenever we call setModulesToEncoders? Should they be called together?
    public void initialize(Pose2d pose, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Update gyro inputs to get current state
        gyroIO.updateInputs(gyroInputs);
        this.swervePoseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            pose.getRotation(),
            drivetrain.getModulePositions(),
            pose
        );
    }

    public void zeroGyro() {
        // also resets the pose estimator, if it has been previously initialized

        Pose2d startingPose = null;
        if (this.swervePoseEstimator != null) {
            startingPose = getPose();
            startingPose = new Pose2d(
                startingPose.getX(),
                startingPose.getY(),
                new Rotation2d(0.0)
            );
        }

        // Reset pose estimator with zeroed heading
        if (startingPose != null && this.drivetrain != null) {
            gyroIO.updateInputs(gyroInputs);
            this.swervePoseEstimator.resetPosition(
                gyroInputs.data.yawPosition(),
                drivetrain.getModulePositions(),
                startingPose
            );
        }
    }

    public void updateInputs() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("PoseEstimator/Gyro", gyroInputs);
    }

    public Pose2d getPose() {
        return this.swervePoseEstimator.getEstimatedPosition();
    }

    public Rotation2d getGyroYaw() {
        return gyroInputs.data.yawPosition();
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        this.swervePoseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void addOdometryMeasurement(SwerveModulePosition[] positions) {
        this.swervePoseEstimator.update(
            gyroInputs.data.yawPosition(),
            positions
        );
    }
}
