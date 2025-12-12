package frc.robot.subsystems.drivetrain;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import java.util.Queue;

public class GyroIORedux implements GyroIO {

    private final Canandgyro gyro = new Canandgyro(30);

    private final Queue<Double> yawTimestampQueue;
    private final Queue<Double> yawPositionQueue;

    public GyroIORedux() {
        // Configure the gyro
        CanandgyroSettings settings = new CanandgyroSettings()
            .setYawFramePeriod(1.0 / DriveConstants.ODOMETRY_FREQUENCY)
            .setAngularPositionFramePeriod(0.01)
            .setAngularVelocityFramePeriod(0.01);
        gyro.setSettings(settings, 0.25, 5);
        gyro.setYaw(0.0, 0.1);
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
}
