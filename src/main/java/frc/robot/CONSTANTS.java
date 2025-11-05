package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class CONSTANTS {
    // Vision Constants
    public static AprilTagFieldLayout APRILTAG_FIELD_LAYOUT 
        = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

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
}
