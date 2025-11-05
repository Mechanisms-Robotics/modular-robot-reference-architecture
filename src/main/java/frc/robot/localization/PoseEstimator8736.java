package frc.robot.localization;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/*
 * TEST PLAN:
 * 
 * Test fieldToRobotChassisSpeeds because of changes to the way we handle gyro angles.
 */

public class PoseEstimator8736 {
    private static final int GYRO_CAN_ID = 9;
    private final Canandgyro gyro = new Canandgyro(GYRO_CAN_ID);

    private SwerveDrivePoseEstimator poseEstimator;

    public void initialize(Pose2d pose) {
        double yawInRotations = pose.getRotation().getDegrees() / 360.0; // between 0.0 and 1.0
        gyro.setYaw(yawInRotations);
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }
    
    // THE ONLY THING THIS CURRENTLY RETURNS IS GYRO HEADING
    public Pose2d getPose() {
        return new Pose2d(0.0, 0.0, gyro.getRotation2d());
    }

    public void addVisionMeasurement(Pose3d pose, double timestampSeconds) {
        // TODO: fill in
    }
}
