// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.CONSTANTS.DriveConstants;
import frc.robot.CONSTANTS.VisionConstants;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FollowPath;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXRedux;
import frc.robot.subsystems.vision.PoseCameraIOPhoton;
import frc.robot.subsystems.vision.PoseCameraIOSim;
import frc.robot.subsystems.vision.Vision;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Owns and wires together every subsystem, the driver controls, and the
 * autonomous options. This is the composition root: hardware vs. simulation
 * IO implementations are chosen here (and only here) based on CURRENT_MODE.
 */
public class RobotContainer {
    public final Drivetrain drivetrain;

    // Never read after construction — the Vision subsystem registers itself
    // with the CommandScheduler and does all its work in periodic(). The
    // field keeps a strong reference and documents ownership.
    @SuppressWarnings("unused")
    private final Vision vision;
    private final DrivetrainController drivetrainController;

    // The chooser holds auto NAMES (not Commands): building the Command is
    // deferred to a Supplier so a fresh instance is constructed each time the
    // selection changes — scheduled commands should not be reused.
    public final SendableChooser<String> autoChooser = new SendableChooser<>();

    private final CommandPS4Controller controller = new CommandPS4Controller(
        CONSTANTS.CONTROLLER_PORT
    );

    private final HashMap<String, Supplier<Command>> autos = new HashMap<>();

    public RobotContainer() {
        if (CONSTANTS.CURRENT_MODE == CONSTANTS.SIM_MODE) {
            this.drivetrain = new Drivetrain(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FRONT_LEFT),
                new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                new ModuleIOSim(DriveConstants.BACK_LEFT),
                new ModuleIOSim(DriveConstants.BACK_RIGHT)
            );

            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOSim(
                    "Photon_Camera_Sim1", 
                    Transform3d.kZero, 
                    drivetrain.poseEstimator
                ));
        } else {
            this.drivetrain = new Drivetrain(
                new GyroIORedux(),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.FRONT_RIGHT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_LEFT),
                new ModuleIOTalonFXRedux(DriveConstants.BACK_RIGHT)
            );

            this.vision = new Vision(
                this.drivetrain.poseEstimator,
                new PoseCameraIOPhoton(
                    VisionConstants.CAMERA1_NAME,
                    VisionConstants.ROBOT_TO_CAMERA1
                ),
                new PoseCameraIOPhoton(
                    VisionConstants.CAMERA2_NAME,
                    VisionConstants.ROBOT_TO_CAMERA2
                )
            );
        }

        this.drivetrainController = new DrivetrainController(this.drivetrain);

        configureBindings();

        // Without this call the chooser is never populated and
        // autoChooser.getSelected() returns null in disabledPeriodic —
        // the "getName() is null" crash this file was hotfixed for.
        publishAutoNames();

        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    private void configureBindings() {
        // Cross (X) re-zeros "forward" to wherever the driver is now facing —
        // the standard fix when field-oriented drive gets skewed mid-match.
        this.controller
            .cross()
            .onTrue(
                new InstantCommand(() -> {
                    this.drivetrain.resetHeading();
                })
            );

        // Default teleop drive: field-oriented, with squared inputs for fine
        // control near center and full speed at the edges.
        this.drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Stick axes are +down/+right; robot axes are +X forward,
                    // +Y left — hence both negations.
                    double forward = -this.controller.getLeftY();
                    double strafe = -this.controller.getLeftX();
                    Translation2d driveSpeeds = getDriveVelocity(
                        forward,
                        strafe
                    );
                    
                    double rotation = -this.controller.getRightX();

                    // apply deadbands and scaling
                    rotation = MathUtil.applyDeadband(
                        rotation,
                        CONSTANTS.DriveConstants.DEADBAND
                    );

                    rotation = Math.copySign(rotation * rotation, rotation);

                    // Scale unitless [-1, 1] stick values to physical speeds.
                    // Max angular rate = max wheel speed at the drivebase
                    // radius (the fastest we can spin without any wheel
                    // exceeding its linear speed limit).
                    ChassisSpeeds speeds = new ChassisSpeeds(
                        driveSpeeds.getX() *
                            CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS.in(
                                MetersPerSecond
                            ),
                        driveSpeeds.getY() *
                            CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS.in(
                                MetersPerSecond
                            ),
                        (rotation *
                                (CONSTANTS.DriveConstants.SPEED_AT_12_VOLTS.in(
                                        MetersPerSecond
                                    ))) /
                            CONSTANTS.DriveConstants.DRIVE_BASE_RADIUS
                    );

                    // convert to robot-oriented coordinates and pass to swerve subsystem
                    ChassisSpeeds robotOriented =
                        this.drivetrainController.fieldToRobotChassisSpeeds(
                            speeds
                        );
                    this.drivetrain.setDesiredState(robotOriented);
                },
                this.drivetrain
            )
        );
    }

    /**
     * Registers every autonomous routine and publishes the chooser. Add new
     * autos to the map here; the chooser and Robot.disabledPeriodic pick them
     * up by name automatically.
     */
    private void publishAutoNames() {
        autos.put("None", () -> Commands.none());

        // Trajectory-free mobility auto: works even if odometry/vision are
        // misbehaving. 1.5 m/s for 2 s = ~3 m off the starting line.
        autos.put("Drive Forward", () ->
            new DriveForward(this.drivetrain, 1.5, 2.0));

        // Choreo path autos: register only the trajectories that actually
        // load, so a missing/renamed .traj file costs us the option instead
        // of crashing robot code on boot.
        registerChoreoAuto("Example Path");

        for (String name : autos.keySet()) {
            autoChooser.addOption(name, name);
        }

        autoChooser.setDefaultOption("None", "None");

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Loads a Choreo trajectory from src/main/deploy/choreo/<name>.traj and,
     * if it exists, registers a FollowPath auto for it under the same name.
     * The first path of an auto resets the pose estimate to its start point.
     */
    private void registerChoreoAuto(String trajectoryName) {
        Optional<Trajectory<SwerveSample>> trajectory =
            Choreo.loadTrajectory(trajectoryName);

        if (trajectory.isPresent()) {
            autos.put(trajectoryName, () ->
                new FollowPath(trajectory.get(), this.drivetrain, true));
        } else {
            System.err.println(
                "[RobotContainer] Choreo trajectory '" + trajectoryName +
                "' not found in deploy/choreo — auto not registered."
            );
        }
    }

    /**
     * Builds a FRESH command for the named auto ("None"/unknown builds a
     * no-op). Called from Robot.disabledPeriodic whenever the drive team
     * changes the selection; the command's name is set to the auto name so
     * callers can detect selection changes by comparing names.
     */
    public Command getAutonomousCommand(String name) {
        Command autoCommand = this.autos.getOrDefault(name, () -> Commands.none()).get();

        autoCommand.setName(name);
        return autoCommand;
    }

    /**
     * Converts raw stick (x, y) into a drive translation direction+magnitude:
     * deadband on the combined magnitude (so diagonal creep is filtered too),
     * then square the magnitude for fine low-speed control while preserving
     * the stick direction exactly.
     */
    private static Translation2d getDriveVelocity(double x, double y) {
        double linearMag = MathUtil.applyDeadband(
            Math.hypot(x, y),
            DriveConstants.DEADBAND
        );
        Rotation2d direction = new Rotation2d(Math.atan2(y, x));
        linearMag = linearMag * linearMag;

        // Build a unit pose facing the stick direction and push it forward by
        // the magnitude — a compact way to get (mag * cos, mag * sin).
        return new Pose2d(Translation2d.kZero, direction)
            .transformBy(new Transform2d(linearMag, 0.0, Rotation2d.kZero))
            .getTranslation();
    }
}
