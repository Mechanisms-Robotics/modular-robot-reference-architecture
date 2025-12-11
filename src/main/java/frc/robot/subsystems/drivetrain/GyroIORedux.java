package frc.robot.subsystems.drivetrain;

import static frc.robot.CONSTANTS.*;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIORedux implements GyroIO {
    private final Canandgyro gyro;

    public GyroIORedux() {
        this.gyro = new Canandgyro(GYRO_CAN_ID);

        // Configure the gyro
        CanandgyroSettings settings =
            new CanandgyroSettings()
                .setYawFramePeriod(0.02)
                .setAngularPositionFramePeriod(0.01)
                .setAngularVelocityFramePeriod(0.01);
        gyro.setSettings(settings, 0.25, 5);
        gyro.setYaw(0.0, 0.1);
        gyro.clearStickyFaults();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.data =
            new GyroIOData(
                gyro.isConnected(),
                Rotation2d.fromRotations(gyro.getYaw()),
                Units.rotationsToRadians(gyro.getAngularVelocityYaw()),
                Rotation2d.fromRotations(gyro.getPitch()),
                Units.rotationsToRadians(gyro.getAngularVelocityPitch()),
                Rotation2d.fromRotations(gyro.getRoll()),
                Units.rotationsToRadians(gyro.getAngularVelocityRoll()));

        // Note: Odometry timestamps and positions are not used in this implementation
        // as this project does not have a high-frequency odometry thread
        inputs.odometryYawTimestamps = new double[] {};
        inputs.odometryYawPositions = new Rotation2d[] {};
    }
}
