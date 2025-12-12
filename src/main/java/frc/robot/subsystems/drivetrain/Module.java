package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Module {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();
    private final int index;
    private final SwerveModuleConstants<
        TalonFXConfiguration,
        TalonFXConfiguration,
        CANcoderConfiguration
    > constants;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions =
        new SwerveModulePosition[] {};

    public Module(
        ModuleIO io,
        int index,
        SwerveModuleConstants<
            TalonFXConfiguration,
            TalonFXConfiguration,
            CANcoderConfiguration
        > constants
    ) {
        this.io = io;
        this.index = index;
        this.constants = constants;
        this.driveDisconnectedAlert = new Alert(
            "Disconnected drive motor on module " + index + ".",
            AlertType.kError
        );
        this.turnDisconnectedAlert = new Alert(
            "Disconnected turn motor on module " + index + ".",
            AlertType.kError
        );
        this.turnEncoderDisconnectedAlert = new Alert(
            "Disconnected turn encoder on module " + index + ".",
            AlertType.kError
        );
    }

    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Drive/Module" + this.index, this.inputs);

        // Calculate positions for odometry
        int sampleCount = this.inputs.odometryTimestamps.length; // All signals are sampled together
        this.odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters =
                this.inputs.odometryDrivePositionsRad[i] *
                this.constants.WheelRadius;
            Rotation2d angle = this.inputs.odometryTurnPositions[i];
            this.odometryPositions[i] = new SwerveModulePosition(
                positionMeters,
                angle
            );
        }

        // Update alerts
        this.driveDisconnectedAlert.set(!this.inputs.driveConnected);
        this.turnDisconnectedAlert.set(!this.inputs.turnConnected);
        this.turnEncoderDisconnectedAlert.set(
            !this.inputs.turnEncoderConnected
        );
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(this.inputs.turnPosition);

        // Apply setpoints
        this.io.setDriveVelocity(
            state.speedMetersPerSecond / this.constants.WheelRadius
        );
        this.io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        this.io.setDriveOpenLoop(output);
        this.io.setTurnPosition(Rotation2d.kZero);
    }

    /** Disables all outputs to motors. */
    public void stop() {
        this.io.setDriveOpenLoop(0.0);
        this.io.setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return this.inputs.turnPosition;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return this.inputs.drivePositionRad * this.constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return this.inputs.driveVelocityRadPerSec * this.constants.WheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return this.odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return this.inputs.odometryTimestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return this.inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(this.inputs.driveVelocityRadPerSec);
    }
}
