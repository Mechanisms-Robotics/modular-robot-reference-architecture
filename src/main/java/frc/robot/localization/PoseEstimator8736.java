package frc.robot.localization;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

public class PoseEstimator8736 {
    private static final int GYRO_CAN_ID = 9;
    private final Canandgyro gyro = new Canandgyro(GYRO_CAN_ID);

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    // TODO: Think about this because it may be better to return a Pose2D or whatever....
    public double getYaw() {
        return gyro.getYaw();
    }
}
