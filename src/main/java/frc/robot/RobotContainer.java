// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.CONSTANTS.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.PoseEstimator8736;
import frc.robot.subsystems.drivetrain.CTREModule;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainController;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SimModule;

public class RobotContainer {

    private final Drivetrain drivetrain;
    private final PoseEstimator8736 poseEstimator;
    private final DrivetrainController drivetrainController;

    private final CommandPS4Controller controller = new CommandPS4Controller(
        CONTROLLER_PORT
    );

    public RobotContainer() {
        // Not entriely happy with how this looks. But I do like passing in the IO
        // as this makes it easy to change hardware without chaning logic.
        if (RobotBase.isReal()) {
            this.poseEstimator = new PoseEstimator8736(new GyroIORedux());
            this.drivetrain = new Drivetrain(
                new CTREModule(
                    FRONT_LEFT_STEERING_CAN_ID,
                    FRONT_LEFT_DRIVE_CAN_ID,
                    FRONT_LEFT_ENCODER_CAN_ID
                ),
                new CTREModule(
                    FRONT_RIGHT_STEERING_CAN_ID,
                    FRONT_RIGHT_DRIVE_CAN_ID,
                    FRONT_RIGHT_ENCODER_CAN_ID
                ),
                new CTREModule(
                    BACK_LEFT_STEERING_CAN_ID,
                    BACK_LEFT_DRIVE_CAN_ID,
                    BACK_LEFT_ENCODER_CAN_ID
                ),
                new CTREModule(
                    BACK_RIGHT_STEERING_CAN_ID,
                    BACK_RIGHT_DRIVE_CAN_ID,
                    BACK_RIGHT_ENCODER_CAN_ID
                )
            );
        } else {
            // For simulation, we need to create the modules and kinematics first
            // so the GyroIOSim can calculate yaw from module states
            SimModule frontLeft = new SimModule();
            SimModule frontRight = new SimModule();
            SimModule backLeft = new SimModule();
            SimModule backRight = new SimModule();

            SwerveDriveKinematics simKinematics = new SwerveDriveKinematics(
                FRONT_LEFT_MODULE_LOCATION,
                FRONT_RIGHT_MODULE_LOCATION,
                BACK_LEFT_MODULE_LOCATION,
                BACK_RIGHT_MODULE_LOCATION
            );

            // Create a temporary drivetrain reference holder for the lambda
            final Drivetrain[] drivetrainHolder = new Drivetrain[1];

            this.drivetrain = new Drivetrain(
                frontLeft,
                frontRight,
                backLeft,
                backRight
            );
            drivetrainHolder[0] = this.drivetrain;

            this.poseEstimator = new PoseEstimator8736(
                new GyroIOSim(simKinematics, () ->
                    drivetrainHolder[0].getModuleStates()
                )
            );
        }
        this.drivetrainController = new DrivetrainController(poseEstimator);
        this.poseEstimator.initialize(new Pose2d(), this.drivetrain);
        this.drivetrain.setPoseEstimator(this.poseEstimator);
        configureBindings();
    }

    public void setSwerveModulesToEncoders() {
        this.drivetrain.setModulesToEncoders();
    }

    public void zeroGyro() {
        this.poseEstimator.zeroGyro();
    }

    public void initializePoseEstimator() {
        this.poseEstimator.initialize(new Pose2d(), this.drivetrain);
    }

    private void configureBindings() {
        controller
            .cross()
            .onTrue(
                new InstantCommand(() -> {
                    this.poseEstimator.zeroGyro();
                })
            );

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
                        forward * forward * forward * MAX_SPEED_METERS_PER_SEC,
                        strafe * strafe * strafe * MAX_SPEED_METERS_PER_SEC,
                        rotation * rotation * rotation * MAX_ANGULAR_RAD_PER_SEC
                    );

                    // convert to robot-oriented coordinates and pass to swerve subsystem
                    ChassisSpeeds robotOriented =
                        drivetrainController.fieldToRobotChassisSpeeds(speeds);
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
