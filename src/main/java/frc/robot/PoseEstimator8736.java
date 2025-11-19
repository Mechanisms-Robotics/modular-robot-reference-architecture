package frc.robot;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;

/*
 * TEST PLAN:
 * 
 * Determine where to initialize the pose estimator.
 * Test output of getModulePositions to make sure they are correct.
 */

public class PoseEstimator8736 {
    private static final int GYRO_CAN_ID = 9;
    private final Canandgyro gyro = new Canandgyro(GYRO_CAN_ID);

    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Drivetrain drivetrain;

    // TODO: Where do we call this from and do we need to make sure drivetrain encoders are set first?
    // Can we just call this whenever we call setModulesToEncoders? Should they be called together?
    public void initialize(Pose2d pose, Drivetrain drivetrain) {
        double yawInRotations = pose.getRotation().getDegrees() / 360.0; // between 0.0 and 1.0
        this.gyro.setYaw(yawInRotations);
        this.drivetrain = drivetrain;
        this.swervePoseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            pose.getRotation(), // initial gyro angle  TODO: Should I use the actual gyro angle instead?
            drivetrain.getModulePositions(),
            pose);
    }

    public void zeroGyro() {
        // also resets the pose estimator, if it has been previously initialized
        
        Pose2d startingPose = null;
        if (this.swervePoseEstimator != null) {
            startingPose = getPose();
            startingPose = new Pose2d(
                startingPose.getX(),
                startingPose.getY(),
                new Rotation2d(0.0));
        }

        this.gyro.setYaw(0.0);

        if (startingPose != null && this.drivetrain != null) {
            initialize(startingPose, this.drivetrain);
        }
    }
    
    public Pose2d getPose() {
        return this.swervePoseEstimator.getEstimatedPosition();
    }

    public double getGyroYaw() {
        return this.gyro.getYaw(); // returns yaw in rotations
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        this.swervePoseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void addOdometryMeasurement(SwerveModulePosition[] positions) {
        this.swervePoseEstimator.update(this.gyro.getRotation2d(), positions);
    }
}
