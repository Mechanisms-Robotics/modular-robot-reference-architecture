package frc.robot.localization;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drivetrain.Drivetrain;

/*
 * TEST PLAN:
 * 
 * Test fieldToRobotChassisSpeeds because of changes to the way we handle gyro angles.
 * Test output of getModulePositions to make sure they are correct.
 */

public class PoseEstimator8736 {
    private static final int GYRO_CAN_ID = 9;
    private final Canandgyro gyro = new Canandgyro(GYRO_CAN_ID);

    private SwerveDrivePoseEstimator poseEstimator;

    public void initialize(Pose2d pose, Drivetrain drivetrain) {
        double yawInRotations = pose.getRotation().getDegrees() / 360.0; // between 0.0 and 1.0
        this.gyro.setYaw(yawInRotations);
        this.poseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            pose.getRotation(), // initial gyro angle
            null, // this is a SwerveModulePosition[] array, but we don't have one here
            pose);
    }

    public void zeroGyro() {
        // TODO: Do we need to do anything with the poseEstimator when we zero the gyro?
        this.gyro.setYaw(0.0);
    }
    
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        this.poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void addOdometryMeasurement(SwerveModulePosition[] positions) {
        this.poseEstimator.update(this.gyro.getRotation2d(), positions);
    }
}
