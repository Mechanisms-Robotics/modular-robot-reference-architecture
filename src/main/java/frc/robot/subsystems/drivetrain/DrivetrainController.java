package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.PoseEstimator8736;

public class DrivetrainController {

    private final PoseEstimator8736 poseEstimator;

    public DrivetrainController(PoseEstimator8736 poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public ChassisSpeeds fieldToRobotChassisSpeeds(
        ChassisSpeeds fieldOriented
    ) {
        Rotation2d angle = poseEstimator.getGyroYaw();

        double cosA = angle.getCos();
        double sinA = angle.getSin();

        double vxRobot =
            fieldOriented.vxMetersPerSecond * cosA +
            fieldOriented.vyMetersPerSecond * sinA;
        double vyRobot =
            -fieldOriented.vxMetersPerSecond * sinA +
            fieldOriented.vyMetersPerSecond * cosA;

        return new ChassisSpeeds(
            vxRobot,
            vyRobot,
            fieldOriented.omegaRadiansPerSecond
        );
    }
}
