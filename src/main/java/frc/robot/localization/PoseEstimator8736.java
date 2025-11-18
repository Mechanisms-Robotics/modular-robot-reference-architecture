package frc.robot.localization;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose3d;
import static frc.robot.CONSTANTS.*;

public class PoseEstimator8736 {
    private final Canandgyro gyro = new Canandgyro(GYRO_CAN_ID);

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    // TODO: Think about this because it may be better to return a Pose2D or whatever....
    public double getYaw() {
        return gyro.getYaw();
    }

    public void addVisionMeasurement(Pose3d pose, double timestampSeconds) {
        // TODO: fill in
    }
}
