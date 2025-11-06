// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.localization.PoseEstimator8736;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;

public class RobotContainer {
  // TODO: This should be set to the robot's actual max speeds and then we should set the controller's
  // response curves. Determine these experimentally.

  private static final double MAX_SPEED_METERS_PER_SEC = 5.0;
  private static final double MAX_ANGULAR_RAD_PER_SEC = 3*Math.PI;
  private static final double DEADBAND = 0.08;

  private static final Translation2d FRONT_LEFT_MODULE_LOCATION = new Translation2d(0.33, 0.23);
  private static final Translation2d FRONT_RIGHT_MODULE_LOCATION = new Translation2d(0.33, -0.23);
  private static final Translation2d BACK_LEFT_MODULE_LOCATION = new Translation2d(-0.33, 0.23);
  private static final Translation2d BACK_RIGHT_MODULE_LOCATION = new Translation2d(-0.33, -0.23);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    FRONT_LEFT_MODULE_LOCATION, FRONT_RIGHT_MODULE_LOCATION,
    BACK_LEFT_MODULE_LOCATION, BACK_RIGHT_MODULE_LOCATION);

  private final Drivetrain drivetrain = new Drivetrain(kinematics);
  private final PoseEstimator8736 poseEstimator = new PoseEstimator8736();
  private final DrivetrainController drivetrainController = new DrivetrainController(poseEstimator);

  private static final int CONTROLLER_PORT = 0;
  private final CommandPS4Controller controller = new CommandPS4Controller(CONTROLLER_PORT);


  public RobotContainer() {
    configureBindings();
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

    drivetrain.setDefaultCommand(
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
              ChassisSpeeds robotOriented = drivetrainController.fieldToRobotChassisSpeeds(speeds);
              drivetrain.setDesiredState(robotOriented);
          },
          drivetrain
      )
  );
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
