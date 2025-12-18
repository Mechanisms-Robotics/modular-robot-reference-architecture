package frc.robot;

import static frc.robot.CONSTANTS.PATH_FOLLOWER_P_THETA;

import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class FollowPath extends Command {

  private final Trajectory<SwerveSample> trajectory;
  private final Drivetrain drivetrain;
  private final PoseEstimator8736 poseEstimator;
  private final boolean resetPose;

  private final Timer timer = new Timer();
  private final HolonomicDriveController holonomicController;
  
  public FollowPath(
      Trajectory<SwerveSample> trajectory,
      Drivetrain drivetrain,
      PoseEstimator8736 poseEstimator,
      boolean resetPose) {

    this.trajectory = trajectory;
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimator;
    this.resetPose = resetPose;

    Constraints thetaProfile = new TrapezoidProfile.Constraints(
        CONSTANTS.MAX_ANGULAR_RAD_PER_SEC, CONSTANTS.MAX_ANGULAR_RAD_PER_SEC * 2);

    ProfiledPIDController thetaController =
        new ProfiledPIDController(CONSTANTS.PATH_FOLLOWER_P_THETA, 0, 0, thetaProfile);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: Tune the controller.
    holonomicController = new HolonomicDriveController(
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

    if (this.resetPose) {
      Optional<Pose2d> initialPose = trajectory.getInitialPose(false); // TODO: Should we mirror for red?
      if (initialPose.isEmpty()) {
        // TODO: Why would this ever happen? Should we handle it differently?
        throw new IllegalStateException("Trajectory has no initial pose!");
      }
      this.drivetrain.setModulesToEncoders();
      this.poseEstimator.initialize(
          initialPose.get(),
          this.drivetrain);
    }
  }

  @Override
  public void execute() {
    double t = this.timer.get();

    Optional<SwerveSample> swerveSample = this.trajectory.sampleAt(
      t, false); // TODO: Should we mirror for red?
    if (swerveSample.isEmpty()) {
      return; // TODO: Why would this ever happen? Should we handle it differently?
    }

    // TODO: This loses the capability of Choreo to control the wheels optimally. See the choreo docs.

    // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html

    ChassisSpeeds sampleSpeeds = swerveSample.get().getChassisSpeeds();

    double desiredLinearVelocity = Math.sqrt(
        sampleSpeeds.vxMetersPerSecond * sampleSpeeds.vxMetersPerSecond +
        sampleSpeeds.vyMetersPerSecond * sampleSpeeds.vyMetersPerSecond);

    ChassisSpeeds commandedSpeeds =
        this.holonomicController.calculate(
            this.poseEstimator.getPose(),
            swerveSample.get().getPose(),
            desiredLinearVelocity,
            swerveSample.get().getPose().getRotation()
        );

    this.drivetrain.setDesiredState(commandedSpeeds);  
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.setDesiredState(new ChassisSpeeds()); // TODO: There may be cases where we don't want the robot to stop!
    this.timer.stop();
  }

  @Override
  public boolean isFinished() {
    // TODO: If precition is important we may need an end-state controller.
    return this.timer.get() >= this.trajectory.getTotalTime(); // TODO: I assume total time is in seconds?
  }  
}