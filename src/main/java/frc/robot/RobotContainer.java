// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.SwerveSample;
import static frc.robot.CONSTANTS.*;

import java.util.Optional;

public class RobotContainer {

  private final Drivetrain drivetrain = new Drivetrain();
  private final PoseEstimator8736 poseEstimator = new PoseEstimator8736();
  private final DrivetrainController drivetrainController = new DrivetrainController(poseEstimator);

  private final CommandPS4Controller controller = new CommandPS4Controller(CONTROLLER_PORT);

  public RobotContainer() {
    // TODO: Think about where to initialize all of this properly
    this.poseEstimator.initialize(new Pose2d(), this.drivetrain);
    this.drivetrain.setPoseEstimator(this.poseEstimator);
    configureBindings();
    generateAutos();
  }

  public void setSwerveModulesToEncoders() {
    this.drivetrain.setModulesToEncoders();
  }

  public void zeroGyro() {
    this.poseEstimator.zeroGyro();
  }

  private void configureBindings() {
    controller.cross().onTrue(new InstantCommand(
      () -> {
        this.poseEstimator.zeroGyro();
      }
    ));

    this.drivetrain.setDefaultCommand(
      new RunCommand(
          () -> {
              double forward = -this.controller.getLeftY(); // Negative to match FRC convention
              double strafe = -this.controller.getLeftX();
              double rotation = -this.controller.getRightX();

              // apply deadbands and scaling

              forward = Math.abs(forward) > DEADBAND ? forward : 0.0;
              strafe = Math.abs(strafe) > DEADBAND ? strafe : 0.0;
              rotation = Math.abs(rotation) > DEADBAND ? rotation : 0.0;

              ChassisSpeeds speeds = new ChassisSpeeds(
                  forward*forward*forward*MAX_SPEED_METERS_PER_SEC,
                  strafe*strafe*strafe* MAX_SPEED_METERS_PER_SEC,
                  rotation*rotation*rotation*MAX_ANGULAR_RAD_PER_SEC
              );

              // convert to robot-oriented coordinates and pass to swerve subsystem
              ChassisSpeeds robotOriented = this.drivetrainController.fieldToRobotChassisSpeeds(speeds);
              this.drivetrain.setDesiredState(robotOriented);
          },
          this.drivetrain
      )
    );
  }

  private Command testAuto = null;

  private void generateAutos() {
    Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("Test Path");
    this.testAuto = new FollowPath(
        trajectory.get(),
        this.drivetrain,
        this.poseEstimator,
        true
    );
  }

  public Command getAutonomousCommand() {
    return testAuto;
    //return Commands.print("No autonomous command configured");
  }
}

