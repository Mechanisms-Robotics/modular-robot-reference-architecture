package frc.robot.subsystems.drivetrain;

import static frc.robot.CONSTANTS.*;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {

    private final SwerveModuleIO io;
    private final ModuleIOInputsAutoLogged inputs =
        new ModuleIOInputsAutoLogged();
    private final String name;

    public SwerveModule(SwerveModuleIO io, String name) {
        this.io = io;
        this.name = name;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + name, inputs);
    }

    public void setModuleToEncoder() {
        this.io.calibrate();
    }

    public void setModuleState(SwerveModuleState state) {
        // Optimize the state to avoid spinning more than 90 degrees
        state = SwerveModuleState.optimize(state, inputs.data.turnPosition());

        // Set the turn motor to the desired angle
        io.runTurnPosition(state.angle);

        // Calculate a speed scale factor (cosine compensation)
        double scaleFactor = state.angle
            .minus(inputs.data.turnPosition())
            .getCos();

        // Convert speed from m/s to rad/s and apply scale factor
        double wheelRadPerSec =
            state.speedMetersPerSecond / WHEEL_RADIUS_METERS;
        io.runDriveVelocity(wheelRadPerSec * scaleFactor, 0.0);
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            inputs.data.drivePositionRad() * WHEEL_RADIUS_METERS,
            inputs.data.turnPosition()
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.data.driveVelocityRadPerSec() * WHEEL_RADIUS_METERS,
            inputs.data.turnPosition()
        );
    }

    public double getPositionMeters() {
        return (inputs.data.drivePositionRad() * WHEEL_RADIUS_METERS);
    }
}
