package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DrivetrainController {
    private final Drivetrain drivetrain;

    public DrivetrainController(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public ChassisSpeeds fieldToRobotChassisSpeeds(ChassisSpeeds fieldOriented)
    {
        Rotation2d angle = drivetrain.getPose().getRotation();

        double cosA = angle.getCos();
        double sinA = angle.getSin();

        double vxRobot = fieldOriented.vxMetersPerSecond*cosA + fieldOriented.vyMetersPerSecond*sinA;
        double vyRobot = -fieldOriented.vxMetersPerSecond*sinA + fieldOriented.vyMetersPerSecond*cosA;
        
        return new ChassisSpeeds(vxRobot, vyRobot, fieldOriented.omegaRadiansPerSecond);
    }
}