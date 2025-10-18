package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CONSTANTS;
import frc.robot.PoseEstimator8736;

public class Drivetrain extends SubsystemBase {

    // Per WPILib documentation +X is forward and +Y is left (oriented to the robot)
    // Positive rotation is counterclockwise
    SwerveDriveKinematics kinematics;
    ChassisSpeeds desiredChassisSpeeds;
    private SwerveModule[] modules = new SwerveModule[4];
    private PoseEstimator8736 poseEstimator;

    private static final Translation2d FRONT_LEFT_MODULE_LOCATION =
        new Translation2d(0.33, 0.23);
    private static final Translation2d FRONT_RIGHT_MODULE_LOCATION =
        new Translation2d(0.33, -0.23);
    private static final Translation2d BACK_LEFT_MODULE_LOCATION =
        new Translation2d(-0.33, 0.23);
    private static final Translation2d BACK_RIGHT_MODULE_LOCATION =
        new Translation2d(-0.33, -0.23);

    /**
     * Remember that the front of the robot is +X and the left side of the robot is
     * +Y.
     */
    public Drivetrain(
        SwerveModuleIO front_left,
        SwerveModuleIO front_right,
        SwerveModuleIO back_left,
        SwerveModuleIO back_right
    ) {
        this.modules[0] = new SwerveModule(front_left, "Front Left");
        this.modules[1] = new SwerveModule(front_right, "Front Right");
        this.modules[2] = new SwerveModule(back_left, "Back Left");
        this.modules[3] = new SwerveModule(back_right, "Back Right");
        this.kinematics = new SwerveDriveKinematics(
            FRONT_LEFT_MODULE_LOCATION,
            FRONT_RIGHT_MODULE_LOCATION,
            BACK_LEFT_MODULE_LOCATION,
            BACK_RIGHT_MODULE_LOCATION
        );
        this.desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    }

    public void setDesiredState(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
    }

    public void setPoseEstimator(PoseEstimator8736 poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public SwerveDriveKinematics getKinematics() {
        return this.kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.modules[0].getModulePosition(),
            this.modules[1].getModulePosition(),
            this.modules[2].getModulePosition(),
            this.modules[3].getModulePosition(),
        };
    }

    public void setModulesToEncoders() {
        for (SwerveModule module : this.modules) {
            module.setModuleToEncoder();
        }
    }

    @Override
    public void periodic() {
        // Update module inputs and logging
        for (SwerveModule module : this.modules) {
            module.periodic();
        }

        // Update pose estimator with current odometry
        if (this.poseEstimator != null) {
            this.poseEstimator.addOdometryMeasurement(getModulePositions());
        }

        // Calculate and set desired module states
        SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(
            this.desiredChassisSpeeds
        );
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            CONSTANTS.MAX_SPEED_METERS_PER_SEC
        );

        for (int i = 0; i < this.modules.length; i++) {
            this.modules[i].setModuleState(moduleStates[i]);
        }
    }
}
