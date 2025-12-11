package frc.robot.subsystems.drivetrain;

import static frc.robot.CONSTANTS.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.Supplier;

public class GyroIOSim implements GyroIO {
    private final SwerveDriveKinematics kinematics;
    private final Supplier<SwerveModuleState[]> moduleStatesSupplier;

    private Rotation2d yawPosition = Rotation2d.kZero;
    private double yawVelocityRadPerSec = 0.0;

    public GyroIOSim(
        SwerveDriveKinematics kinematics,
        Supplier<SwerveModuleState[]> moduleStatesSupplier
    ) {
        this.kinematics = kinematics;
        this.moduleStatesSupplier = moduleStatesSupplier;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // Calculate yaw velocity from module states using kinematics
        SwerveModuleState[] moduleStates = moduleStatesSupplier.get();
        var chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);
        yawVelocityRadPerSec = chassisSpeeds.omegaRadiansPerSecond;

        // Integrate yaw position
        yawPosition = yawPosition.plus(
            Rotation2d.fromRadians(yawVelocityRadPerSec * LOOP_PERIOD_SECS)
        );

        inputs.data =
            new GyroIOData(
                true, // connected
                yawPosition,
                yawVelocityRadPerSec,
                Rotation2d.kZero, // pitchPosition (not simulated)
                0.0, // pitchVelocityRadPerSec
                Rotation2d.kZero, // rollPosition (not simulated)
                0.0 // rollVelocityRadPerSec
            );

        inputs.odometryYawTimestamps = new double[] {};
        inputs.odometryYawPositions = new Rotation2d[] {};
    }
}
