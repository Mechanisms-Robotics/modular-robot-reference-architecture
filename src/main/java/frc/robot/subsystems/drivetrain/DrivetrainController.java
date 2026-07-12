package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.FieldUtil;

/**
 * Translates driver-centric commands into robot-centric ones. Sits between the
 * teleop code in RobotContainer and the Drivetrain subsystem so the coordinate
 * gymnastics live in one place.
 */
public class DrivetrainController {

    private final Drivetrain drivetrain;

    public DrivetrainController(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Converts field-oriented (driver-perspective) chassis speeds into the
     * robot-oriented speeds the drivetrain consumes.
     *
     * <p>Field coordinates are blue-origin, so a red-alliance driver stands at
     * the opposite end of the field. Rotating our heading by 180 degrees keeps
     * "push the stick away from you" meaning "drive away from you" regardless
     * of alliance.
     */
    public ChassisSpeeds fieldToRobotChassisSpeeds(
        ChassisSpeeds fieldOriented
    ) {
        Rotation2d driverRelativeHeading = FieldUtil.isRedAlliance()
            ? this.drivetrain.getPose().getRotation().rotateBy(
                  Rotation2d.k180deg
              )
            : this.drivetrain.getPose().getRotation();

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldOriented,
            driverRelativeHeading
        );
    }
}
