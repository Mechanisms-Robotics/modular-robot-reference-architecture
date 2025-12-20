package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.CONSTANTS.*;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CONSTANTS;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();
    private final String name;

    public SwerveModule(ModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        // Poll for new hardware inputs
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * CONSTANTS.DriveConstants.WHEEL_RADIUS.in(Meters),
            inputs.turnPosition
        );
    }

    public void setModuleState(SwerveModuleState state) {
        // get the current position and optimize the state
        state.optimize(inputs.turnPosition);

        // calculate a speed scale factor (cosine compensation)
        double scaleFactor = state.angle.minus(inputs.turnPosition).getCos();

        // set the drive velocity (convert m/s to rad/s)
        double driveVelocityRadPerSec =
            (state.speedMetersPerSecond * scaleFactor) / DriveConstants.WHEEL_RADIUS.in(Meters);
        io.setDriveVelocity(driveVelocityRadPerSec);

        // set the turn position
        io.setTurnPosition(state.angle);
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public SwerveModulePosition[] getOdometryPositions() {
        int sampleCount = inputs.odometryDrivePositionsRad.length;
        SwerveModulePosition[] positions =
            new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            positions[i] = new SwerveModulePosition(
                inputs.odometryDrivePositionsRad[i] * DriveConstants.WHEEL_RADIUS.in(Meters),
                inputs.odometryTurnPositions[i]
            );
        }
        return positions;
    }
}
