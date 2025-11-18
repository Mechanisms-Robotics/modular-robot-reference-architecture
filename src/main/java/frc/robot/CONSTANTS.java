package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class CONSTANTS {
    //RobotContainer
    public static final int CONTROLLER_PORT = 0;
    
    // Vision Constants
    public static AprilTagFieldLayout APRILTAG_FIELD_LAYOUT 
        = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    //Drivetrain
    public static final Translation2d FRONT_LEFT_MODULE_LOCATION 
        = new Translation2d(0.33, 0.23);
    public static final Translation2d FRONT_RIGHT_MODULE_LOCATION 
        = new Translation2d(0.33, -0.23);
    public static final Translation2d BACK_LEFT_MODULE_LOCATION 
        = new Translation2d(-0.33, 0.23);
    public static final Translation2d BACK_RIGHT_MODULE_LOCATION 
        = new Translation2d(-0.33, -0.23);

    public static final int FRONT_LEFT_STEERING_CAN_ID = 5;
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
    public static final int FRONT_LEFT_ENCODER_CAN_ID = 1;
  
    public static final int FRONT_RIGHT_STEERING_CAN_ID = 6;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 2;
    public static final int FRONT_RIGHT_ENCODER_CAN_ID = 2;

    public static final int BACK_LEFT_STEERING_CAN_ID = 4;
    public static final int BACK_LEFT_DRIVE_CAN_ID = 8;
    public static final int BACK_LEFT_ENCODER_CAN_ID = 4;

    public static final int BACK_RIGHT_STEERING_CAN_ID = 3;
    public static final int BACK_RIGHT_DRIVE_CAN_ID = 7;
    public static final int BACK_RIGHT_ENCODER_CAN_ID = 3;

    // TODO: This should be set to the robot's actual max speeds and then we should set the controller's
    // response curves. Determine these experimentally.
    public static final double MAX_SPEED_METERS_PER_SEC = 5.0;
    public static final double MAX_ANGULAR_RAD_PER_SEC = 3*Math.PI;
    public static final double DEADBAND = 0.08;

    public final static long SWERVE_ENCODER_SET_FREQUECY_SECONDS = 1;

    public static final double WHEEL_RADIUS_METERS = 0.0508; // from the internet
    public static final double STEERING_GEAR_RATIO = 150.0/7.0; // from SDS website
    public static final double DRIVE_GEAR_RATIO = 6.75; // L2 from SDS website

    //Localization
    public static final int GYRO_CAN_ID = 9;

}


