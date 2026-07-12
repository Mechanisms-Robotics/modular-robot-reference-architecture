package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.CONSTANTS.DriveConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import frc.robot.CONSTANTS;

/**
 * One swerve corner: a drive motor, a steer (turn) motor, and an absolute
 * azimuth encoder, hidden behind a {@link ModuleIO}.
 *
 * <p>This class is deliberately hardware-agnostic — everything here is in
 * WPILib-native units (meters, radians). Unit conversion from motor rotations
 * happens in the IO implementations. The Drivetrain calls {@link #periodic()}
 * once per loop (under the odometry lock) and then commands each module via
 * {@link #setModuleState}.
 */
public class SwerveModule {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();
    private final String moduleName; // e.g. "Front Left"; used in log keys

    public SwerveModule(ModuleIO io, String name) {
        this.io = io; // may be real or simulated
        this.moduleName = name;
    }

    public void periodic() {
        // Poll for new hardware inputs, which will be stored in this.inputs
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Drive/Module " + moduleName, this.inputs);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * CONSTANTS.DriveConstants.WHEEL_RADIUS.in(Meters),
            inputs.turnPosition
        );
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            inputs.driveVelocityRadPerSec * DriveConstants.WHEEL_RADIUS.in(Meters),
            inputs.turnPosition
        );
    }

    public void setModuleState(SwerveModuleState state) {
        // Optimize: if the target angle is more than 90 degrees away, flip
        // the wheel 180 and drive backwards instead — never rotate the
        // azimuth further than a quarter turn.
        state.optimize(inputs.turnPosition);

        // Cosine compensation: while the wheel is still rotating toward its
        // setpoint, only the component of its velocity along the target
        // direction is useful — scale speed down by cos(error) so a
        // mid-rotation wheel doesn't drag the robot sideways.
        double scaleFactor = state.angle.minus(inputs.turnPosition).getCos();

        // set the drive velocity (convert m/s to rad/s)
        double driveVelocityRadPerSec =
            (state.speedMetersPerSecond * scaleFactor) / DriveConstants.WHEEL_RADIUS.in(Meters);
        Logger.recordOutput("Module " + this.moduleName + "/Desired Drive Radians Per Second", driveVelocityRadPerSec);
        this.io.setDriveVelocity(driveVelocityRadPerSec);

        // set the turn position
        Logger.recordOutput("Module " + this.moduleName + "/Optimised Angle", state.angle);
        this.io.setTurnPosition(state.angle);
    }

    /** Runs the module with the specified output while controlling to zero degrees. */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
    }

    public double[] getOdometryTimestamps() {
        return this.inputs.odometryTimestamps;
    }

    /**
     * Returns all high-frequency odometry samples captured since the last
     * loop (one per PhoenixOdometryThread tick), converted from wheel radians
     * to meters traveled. Index-aligned with {@link #getOdometryTimestamps}.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        int sampleCount = this.inputs.odometryDrivePositionsRad.length;
        SwerveModulePosition[] positions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            positions[i] = new SwerveModulePosition(
                this.inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RADIUS.in(Meters),
                this.inputs.odometryTurnPositions[i]
            );
        }
        return positions;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRad;
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
    }
}
