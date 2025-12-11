package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public GyroIOData data =
            new GyroIOData(false, Rotation2d.kZero, 0, Rotation2d.kZero, 0, Rotation2d.kZero, 0);
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public record GyroIOData(
        boolean connected,
        Rotation2d yawPosition,
        double yawVelocityRadPerSec,
        Rotation2d pitchPosition,
        double pitchVelocityRadPerSec,
        Rotation2d rollPosition,
        double rollVelocityRadPerSec
    ) {}

    public default void updateInputs(GyroIOInputs inputs) {}
}
