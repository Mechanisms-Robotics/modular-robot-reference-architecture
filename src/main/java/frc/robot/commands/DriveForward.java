package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Dead-simple autonomous: drive straight (robot-relative +X) at a fixed speed
 * for a fixed time, then stop.
 *
 * <p>This exists for two reasons: it's a mobility auto that works with zero
 * trajectory files or field setup, and it's the canonical example of how to
 * write and register an auto (see RobotContainer.publishAutoNames). It
 * deliberately uses no pose estimation — if odometry or vision is broken,
 * this still moves the robot off the line.
 */
public class DriveForward extends Command {

    private final Drivetrain drivetrain;
    private final double speedMetersPerSec;
    private final double durationSeconds;

    private final Timer timer = new Timer();

    public DriveForward(
            Drivetrain drivetrain,
            double speedMetersPerSec,
            double durationSeconds) {
        this.drivetrain = drivetrain;
        this.speedMetersPerSec = speedMetersPerSec;
        this.durationSeconds = durationSeconds;

        super.addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Robot-relative: +X is robot-forward regardless of field heading.
        this.drivetrain.setDesiredState(
            new ChassisSpeeds(this.speedMetersPerSec, 0.0, 0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Desired speeds persist until replaced (see Drivetrain), so an auto
        // MUST command zero on the way out or the robot keeps driving.
        this.drivetrain.setDesiredState(new ChassisSpeeds());
        this.timer.stop();
    }

    @Override
    public boolean isFinished() {
        return this.timer.get() >= this.durationSeconds;
    }
}
