package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PoseEstimator8736;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowPath extends Command {

  private final Trajectory trajectory;
  private final Drivetrain drivetrain;
  private final PoseEstimator8736 poseEstimator;
  private final boolean resetPose;

  private final Timer timer = new Timer();
  private final HolonomicDriveController holonomic;

  // ---- Tunable constants ----
  private static final double kPX = 1.0;
  private static final double kPY = 1.0;
  private static final double kPTheta = 3.0;

  private static final double kMaxAngularVel = Math.PI;        // rad/s
  private static final double kMaxAngularAccel = Math.PI * 2;  // rad/s²
  // ----------------------------

  public FollowPath(
      Trajectory trajectory,
      Drivetrain drivetrain,
      PoseEstimator8736 poseEstimator,
      boolean resetPose) {

    this.trajectory = trajectory;
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;
    this.resetPose = resetPose;

    var thetaProfile = new TrapezoidProfile.Constraints(
        kMaxAngularVel, kMaxAngularAccel);

    var thetaController =
        new ProfiledPIDController(kPTheta, 0, 0, thetaProfile);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    holonomic = new HolonomicDriveController(
        new PIDController(kPX, 0, 0),
        new PIDController(kPY, 0, 0),
        thetaController
    );
  }

  // ------------------------
  // COMMAND LIFECYCLE (2025)
  // ------------------------

  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    if (resetPose) {
      drivetrain.setModulesToEncoders();
      poseEstimator.initialize(
          trajectory.getInitialPose(),
          drivetrain);
    }
  }

  @Override
  public void execute() {

    double t = timer.get();
    Trajectory.State goal = trajectory.sample(t);
    Pose2d currentPose = poseEstimator.getPose();

    // Follow the trajectory’s rotation
    Rotation2d desiredHeading = goal.poseMeters.getRotation();

    ChassisSpeeds commandedSpeeds =
        holonomic.calculate(
            currentPose,
            goal.poseMeters,
            goal.velocityMetersPerSecond,
            desiredHeading
        );

    drivetrain.setDesiredState(commandedSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setDesiredState(new ChassisSpeeds());
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() >= trajectory.getTotalTimeSeconds();
  }

  // Required for the Command interface:
  @Override
  public java.util.Set<edu.wpi.first.wpilibj2.command.Subsystem> getRequirements() {
    return java.util.Set.of(drivetrain);
  }
}
