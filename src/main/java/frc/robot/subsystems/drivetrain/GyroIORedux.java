package frc.robot.subsystems.drivetrain;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.CONSTANTS.Timeouts;
import java.util.Queue;

/**
 * GyroIO for the Redux Canandgyro. Yaw is registered with the
 * PhoenixOdometryThread as a generic (supplier-based) signal so heading
 * samples line up with the high-frequency wheel odometry samples.
 * Redux devices buffer their CAN frames internally, so reads here are
 * non-blocking.
 */
public class GyroIORedux implements GyroIO {

    private final Canandgyro gyro = new Canandgyro(CONSTANTS.GYRO_CAN_ID);

    private final Queue<Double> yawTimestampQueue;
    private final Queue<Double> yawPositionQueue;

    public GyroIORedux() {
        // Configure the gyro
        // Redux setters take a PERIOD in seconds (unlike Phoenix, which takes
        // a frequency in Hz). Yaw runs at odometry rate for the high-frequency
        // odometry thread; the rest run at the standard gyro frame rate.
        CanandgyroSettings settings = new CanandgyroSettings()
            .setYawFramePeriod(1.0 / DriveConstants.ODOMETRY_FREQUENCY)
            .setAngularPositionFramePeriod(
                DriveConstants.GYRO_CAN_FRAME_PERIOD_SEC
            )
            .setAngularVelocityFramePeriod(
                DriveConstants.GYRO_CAN_FRAME_PERIOD_SEC
            );
        gyro.setSettings(
            settings,
            Timeouts.STD_TIMEOUT_LONG,
            Timeouts.STD_RETRY_ATTEMPTS
        );
        gyro.setYaw(0.0);
        gyro.clearStickyFaults();

        // Register the gyro signals
        yawTimestampQueue =
            PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(
            gyro::getYaw
        );
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromRotations(gyro.getYaw());
        inputs.yawVelocityRadPerSec = Units.rotationsToRadians(
            gyro.getAngularVelocityYaw()
        );

        inputs.odometryYawTimestamps = yawTimestampQueue
            .stream()
            .mapToDouble((Double value) -> value)
            .toArray();
        inputs.odometryYawPositions = yawPositionQueue
            .stream()
            .map(Rotation2d::fromRotations)
            .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void zeroGyro() {
        this.gyro.setYaw(0.0);
    }
}
