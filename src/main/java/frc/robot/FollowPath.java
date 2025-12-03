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
        CONSTANTS.MAX_ANGULAR_RAD_PER_SEC, CONSTANTS.MAX_ANGULAR_RAD_PER_SEC * 2);

    var thetaController =
        new ProfiledPIDController(CONSTANTS.PATH_FOLLOWER_P_THETA, 0, 0, thetaProfile);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Tune the controller.
    holonomic = new HolonomicDriveController(
        new PIDController(CONSTANTS.PATH_FOLLOWER_P_X, 0, 0),
        new PIDController(CONSTANTS.PATH_FOLLOWER_P_Y, 0, 0),
        thetaController
    );

    super.addRequirements(drivetrain);
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
    drivetrain.setDesiredState(new ChassisSpeeds()); // TODO: There may be cases where we don't want the robot to stop!
    timer.stop();
  }

  @Override
  public boolean isFinished() { // TODO: If precition is important we may need an end-state controller.
    return timer.get() >= trajectory.getTotalTimeSeconds(); 
  }  
}