package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    public static class ModuleIOInputs {

        public ModuleIOData data = new ModuleIOData(
            false,
            0,
            0,
            0,
            0,
            0,
            false,
            false,
            Rotation2d.kZero,
            Rotation2d.kZero,
            0,
            0,
            0,
            0
        );

        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public record ModuleIOData(
        boolean driveConnected,
        double drivePositionRad,
        double driveVelocityRadPerSec,
        double driveAppliedVolts,
        double driveSupplyCurrentAmps,
        double driveTorqueCurrentAmps,
        boolean turnConnected,
        boolean turnEncoderConnected,
        Rotation2d turnAbsolutePosition,
        Rotation2d turnPosition,
        double turnVelocityRadPerSec,
        double turnAppliedVolts,
        double turnSupplyCurrentAmps,
        double turnTorqueCurrentAmps
    ) {}

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void calibrate() {}

    public default void runDriveOpenLoop(double output) {}

    public default void runTurnOpenLoop(double output) {}

    public default void runDriveVelocity(
        double velocityRadPerSec,
        double feedforward
    ) {}

    public default void runTurnPosition(Rotation2d rotation) {}

    public default void setDrivePID(double kP, double kI, double kD) {}

    public default void setTurnPID(double kP, double kI, double kD) {}

    public default void setBrakeMode(boolean enabled) {}
}
