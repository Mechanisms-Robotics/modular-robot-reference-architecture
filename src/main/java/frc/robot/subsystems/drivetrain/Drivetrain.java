package frc.robot.subsystems.drivetrain;

import static frc.robot.CONSTANTS.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;

public class Drivetrain extends SubsystemBase {

    // Per WPILib documentation +X is forward and +Y is left (oriented to the robot)
    // Positive rotation is counterclockwise

    SwerveDriveKinematics kinematics;
    ChassisSpeeds desiredChassisSpeeds;
    PoseEstimator8736 poseEstimator;
    //private final StructArrayPublisher<SwerveModuleState> publisher;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    /**
     * Remember that the front of the robot is +X and the left side of the robot is
     * +Y.
     */
    public Drivetrain(
        ModuleIO frontLeftModuleIO,
        ModuleIO frontRightModuleIO,
        ModuleIO backLeftModuleIO,
        ModuleIO backRightModuleIO
    ) {
        this.frontLeftModule = new SwerveModule(
            frontLeftModuleIO,
            "Front Left"
        );
        this.frontRightModule = new SwerveModule(
            frontRightModuleIO,
            "Front Right"
        );
        this.backLeftModule = new SwerveModule(backLeftModuleIO, "Back Left");
        this.backRightModule = new SwerveModule(
            backRightModuleIO,
            "Back Right"
        );

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

    public ChassisSpeeds getDesiredState() {
        return this.desiredChassisSpeeds;
    }

    public SwerveDriveKinematics getKinematics() {
        return this.kinematics;
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeftModule.getModulePosition(),
            this.frontRightModule.getModulePosition(),
            this.backLeftModule.getModulePosition(),
            this.backRightModule.getModulePosition(),
        };
    }

    public void setModulesToEncoders() {
        this.frontLeftModule.setModuleToEncoder();
        this.frontRightModule.setModuleToEncoder();
        this.backLeftModule.setModuleToEncoder();
        this.backRightModule.setModuleToEncoder();
    }

    public void setPoseEstimator(PoseEstimator8736 poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    @Override
    public void periodic() {
        // update the pose estimator

        this.poseEstimator.addOdometryMeasurement(this.getModulePositions());

        Pose2d pose = this.poseEstimator.getPose();

        // send the new desired states down to the modules
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
            this.desiredChassisSpeeds
        );

        SwerveModuleState frontLeftState = moduleStates[0];
        SwerveModuleState frontRightState = moduleStates[1];
        SwerveModuleState backLeftState = moduleStates[2];
        SwerveModuleState backRightState = moduleStates[3];

        this.frontLeftModule.setModuleState(frontLeftState);
        this.frontRightModule.setModuleState(frontRightState);
        this.backLeftModule.setModuleState(backLeftState);
        this.backRightModule.setModuleState(backRightState);
    }

    @Override
    public void simulationPeriodic() {
        this.periodic();
    }
}
