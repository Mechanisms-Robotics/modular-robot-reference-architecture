package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.CONSTANTS;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.PoseEstimator8736;
import frc.robot.util.FieldUtil;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The swerve drivetrain subsystem.
 *
 * <p>Architecture: this class owns four {@link SwerveModule}s and a
 * {@link GyroIO}, each of which reads hardware (or simulation) through the
 * AdvantageKit IO layer. Callers hand us a desired {@link ChassisSpeeds} via
 * {@link #setDesiredState}; every loop {@link #periodic()} pulls fresh sensor
 * inputs, feeds the pose estimator, and pushes per-module setpoints back down.
 *
 * <p>Coordinates follow WPILib conventions: +X is robot-forward, +Y is
 * robot-left, and positive rotation is counterclockwise. Field coordinates
 * have their origin at the blue alliance corner.
 */
public class Drivetrain extends SubsystemBase {

    SwerveDriveKinematics kinematics;

    // The most recent commanded chassis speeds (robot-relative). periodic()
    // converts these to module states each loop, so a stale command keeps
    // being applied until someone commands otherwise — commands must
    // explicitly send zero speeds to stop the robot.
    ChassisSpeeds desiredChassisSpeeds;
    
    // Public so RobotContainer can hand it to the Vision subsystem and
    // FollowPath can toggle vision fusion. Could grow a getter later.
    public final PoseEstimator8736 poseEstimator;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final GyroIO gyroIO; // may be a real gyro or a simulated gyro
    private final GyroIOInputsAutoLogged gyroInputs =
        new GyroIOInputsAutoLogged();

    // Guards everything shared with PhoenixOdometryThread: the thread fills
    // per-signal sample queues at 100-250 Hz while periodic() drains them at
    // 50 Hz. Hold this only while reading/writing inputs — never across
    // control logic — and ALWAYS unlock (a missing unlock here once starved
    // the odometry thread forever).
    public static final Lock odometryLock = new ReentrantLock();

    // True: periodic() runs the modules closed-loop toward
    // desiredChassisSpeeds. False: a characterization routine has taken over
    // and is driving the modules open-loop directly (see runCharacterization).
    private boolean driveClosedLoop = true;

    /**
     * Remember that the forward direction of the robot is +X and the left direction is
     * +Y. So if you give a ChassisSpeed of (+1, +1) the robot should translate diagonally
     * forward and left. This also means that the swerve module positions are based
     * on this coordinate system.
     */

    public Drivetrain(
        GyroIO gyroIO,
        ModuleIO frontLeftModuleIO,
        ModuleIO frontRightModuleIO,
        ModuleIO backLeftModuleIO,
        ModuleIO backRightModuleIO
    ) {
        this.gyroIO = gyroIO;

        this.frontLeftModule = new SwerveModule(
            frontLeftModuleIO,
            "Front Left"
        );
        this.frontRightModule = new SwerveModule(
            frontRightModuleIO,
            "Front Right"
        );
        this.backLeftModule = new SwerveModule(
            backLeftModuleIO, 
            "Back Left"
        );
        this.backRightModule = new SwerveModule(
            backRightModuleIO,
            "Back Right"
        );

        this.kinematics = new SwerveDriveKinematics(
            new Translation2d(
                DriveConstants.FRONT_LEFT.LocationX,
                DriveConstants.FRONT_LEFT.LocationY
            ),
            new Translation2d(
                DriveConstants.FRONT_RIGHT.LocationX,
                DriveConstants.FRONT_RIGHT.LocationY
            ),
            new Translation2d(
                DriveConstants.BACK_LEFT.LocationX,
                DriveConstants.BACK_LEFT.LocationY
            ),
            new Translation2d(
                DriveConstants.BACK_RIGHT.LocationX,
                DriveConstants.BACK_RIGHT.LocationY
            )
        );

        // Start the high-frequency odometry sampler. By this point every
        // module IO (and the gyro IO) has already registered its signals in
        // its constructor — start() is a no-op if nothing registered, which
        // is exactly what happens in simulation.
        PhoenixOdometryThread.getInstance().start();

        this.poseEstimator = new PoseEstimator8736(
            this.kinematics,
            Rotation2d.kZero,
            Pose2d.kZero
        );

        this.desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    /**
     * Commands the drivetrain to the given ROBOT-RELATIVE chassis speeds.
     * Field-relative callers should convert first (see DrivetrainController).
     * The command persists until replaced; send zero speeds to stop.
     */
    public void setDesiredState(ChassisSpeeds desiredChassisSpeeds) {
        this.driveClosedLoop = true; // take back control from characterization
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    public ChassisSpeeds getDesiredState() {
        return this.desiredChassisSpeeds;
    }

    public SwerveDriveKinematics getKinematics() {
        return this.kinematics;
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[] {
            this.frontLeftModule.getModuleState(),
            this.frontRightModule.getModuleState(),
            this.backLeftModule.getModuleState(),
            this.backRightModule.getModuleState(),
        };
    }

    @AutoLogOutput(key = "Drivetrain/Velocity")
    public ChassisSpeeds getVelocity() {
        return this.kinematics.toChassisSpeeds(this.getStates());
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeftModule.getModulePosition(),
            this.frontRightModule.getModulePosition(),
            this.backLeftModule.getModulePosition(),
            this.backRightModule.getModulePosition(),
        };
    }

    @Override
    public void periodic() {
        Drivetrain.odometryLock.lock();

        // gyroIO may be a real gyro or it may be simulated.
        // These lines update gyroInputs from gyroIO
        // gyroInputs will be used below
        this.gyroIO.updateInputs(this.gyroInputs);
        Logger.processInputs("Drive/Gyro", this.gyroInputs);

        // calling SwerveModule.periodic() updates each module's inputs from the hardware or simulation
        this.frontLeftModule.periodic();
        this.frontRightModule.periodic();
        this.backLeftModule.periodic();
        this.backRightModule.periodic();

        Drivetrain.odometryLock.unlock();

        // Send updated odometry to the pose estimator
        // All signals are sampled together so only need timestamps from one
        double[] sampleTimestamps =
            this.frontLeftModule.getOdometryTimestamps();
        for (int i = 0; i < sampleTimestamps.length; i++) {
            // Read wheel positions from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            modulePositions[0] = this.frontLeftModule.getOdometryPositions()[i];
            modulePositions[1] = this.frontRightModule.getOdometryPositions()[i];
            modulePositions[2] = this.backLeftModule.getOdometryPositions()[i];
            modulePositions[3] = this.backRightModule.getOdometryPositions()[i];

            // Update pose estimator with gyro rotation (or null to use kinematics only)
            Rotation2d gyroRotation = this.gyroInputs.connected
                ? this.gyroInputs.odometryYawPositions[i]
                : null;
            this.poseEstimator.updateOdometry(
                modulePositions,
                gyroRotation,
                sampleTimestamps[i]
            );
        }
        if (this.driveClosedLoop) {

            // Send the new desired states down to the modules.
            // discretize() compensates for the fact that we only update
            // setpoints every 20 ms: holding a constant (vx, vy, omega) for a
            // whole period makes the robot arc sideways while rotating, so it
            // reshapes the command such that the robot lands where the
            // continuous-time command would have. It was previously computed
            // but not fed into the kinematics, which caused translation drift
            // whenever the robot spun while driving.
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(
                this.desiredChassisSpeeds,
                CONSTANTS.ROBOT_LOOP_PERIOD
            );

            SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
                discreteSpeeds
            );

            SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates,
                CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS
            );

            Logger.recordOutput("SwerveStates/Setpoints", moduleStates);
            Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

            SwerveModuleState frontLeftState = moduleStates[0];
            SwerveModuleState frontRightState = moduleStates[1];
            SwerveModuleState backLeftState = moduleStates[2];
            SwerveModuleState backRightState = moduleStates[3];

            this.frontLeftModule.setModuleState(frontLeftState);
            this.frontRightModule.setModuleState(frontRightState);
            this.backLeftModule.setModuleState(backLeftState);
            this.backRightModule.setModuleState(backRightState);
        }
    }

    /**
     * Re-zeros the robot's heading so "away from the driver" becomes the new
     * forward, keeping the translation estimate. On red the driver faces the
     * -X field direction (blue-origin coordinates), so their "forward" is a
     * 180 degree field heading.
     */
    public void resetHeading() {
        resetPose(new Pose2d(
            getPose().getTranslation(),
            FieldUtil.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero
        ));
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose, getModulePositions());
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPose();
    }

    // NOTE: deliberately no simulationPeriodic() override. The scheduler
    // already calls periodic() in simulation; an override that called
    // periodic() again ran the module physics twice per loop, so the sim
    // robot moved at 2x speed and odometry was double-fed.

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        this.driveClosedLoop = false;
        this.frontLeftModule.runCharacterization(output);
        this.frontRightModule.runCharacterization(output);
        this.backLeftModule.runCharacterization(output);
        this.backRightModule.runCharacterization(output);
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        output += this.frontLeftModule.getFFCharacterizationVelocity();
        output += this.frontRightModule.getFFCharacterizationVelocity();
        output += this.backLeftModule.getFFCharacterizationVelocity();
        output += this.backRightModule.getFFCharacterizationVelocity();
        output /= 4;
        return output;
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        values[0] = this.frontLeftModule.getWheelRadiusCharacterizationPosition();
        values[1] = this.frontRightModule.getWheelRadiusCharacterizationPosition();
        values[2] = this.backLeftModule.getWheelRadiusCharacterizationPosition();
        values[3] = this.backRightModule.getWheelRadiusCharacterizationPosition();
        return values;
    }

    /** Returns the raw gyro rotation read by the IMU */
    public Rotation2d getGyroRotation() {
        return gyroInputs.yawPosition;
    }
}
