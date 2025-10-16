package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  // Per WPILib documentation +X is forward and +Y is left (oriented to the robot)
  // Positive rotation is counterclockwise
  
  private static final Translation2d FRONT_LEFT_MODULE_LOCATION = new Translation2d(0.33, 0.23);
  private static final Translation2d FRONT_RIGHT_MODULE_LOCATION = new Translation2d(0.33, -0.23);
  private static final Translation2d BACK_LEFT_MODULE_LOCATION = new Translation2d(-0.33, 0.23);
  private static final Translation2d BACK_RIGHT_MODULE_LOCATION = new Translation2d(-0.33, -0.23);

  private static final int FRONT_LEFT_STEERING_CAN_ID = 5;
  private static final int FRONT_LEFT_DRIVE_CAN_ID = 1;
  private static final int FRONT_LEFT_ENCODER_CAN_ID = 1;
  
  private static final int FRONT_RIGHT_STEERING_CAN_ID = 6;
  private static final int FRONT_RIGHT_DRIVE_CAN_ID = 2;
  private static final int FRONT_RIGHT_ENCODER_CAN_ID = 2;

  private static final int BACK_LEFT_STEERING_CAN_ID = 4;
  private static final int BACK_LEFT_DRIVE_CAN_ID = 8;
  private static final int BACK_LEFT_ENCODER_CAN_ID = 4;

  private static final int BACK_RIGHT_STEERING_CAN_ID = 3;
  private static final int BACK_RIGHT_DRIVE_CAN_ID = 7;
  private static final int BACK_RIGHT_ENCODER_CAN_ID = 3;

  SwerveDriveKinematics kinematics;
  ChassisSpeeds desiredChassisSpeeds;
  private final StructArrayPublisher<SwerveModuleState> publisher;

  private final SwerveModule frontLeftModule = new SwerveModule(
      FRONT_LEFT_STEERING_CAN_ID, FRONT_LEFT_DRIVE_CAN_ID, FRONT_LEFT_ENCODER_CAN_ID);

  private final SwerveModule frontRightModule = new SwerveModule(
      FRONT_RIGHT_STEERING_CAN_ID, FRONT_RIGHT_DRIVE_CAN_ID, FRONT_RIGHT_ENCODER_CAN_ID);

  private final SwerveModule backLeftModule = new SwerveModule(
      BACK_LEFT_STEERING_CAN_ID, BACK_LEFT_DRIVE_CAN_ID, BACK_LEFT_ENCODER_CAN_ID);

  private final SwerveModule backRightModule = new SwerveModule(
    BACK_RIGHT_STEERING_CAN_ID, BACK_RIGHT_DRIVE_CAN_ID, BACK_RIGHT_ENCODER_CAN_ID);

  /**
   * Remember that the front of the robot is +X and the left side of the robot is
   * +Y.
   */
  public Drivetrain() {

    this.kinematics = new SwerveDriveKinematics(
        FRONT_LEFT_MODULE_LOCATION, FRONT_RIGHT_MODULE_LOCATION,
        BACK_LEFT_MODULE_LOCATION, BACK_RIGHT_MODULE_LOCATION);

    this.desiredChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.publisher = NetworkTableInstance.getDefault().getStructArrayTopic(
        "/SwerveStates", SwerveModuleState.struct).publish();
  }

  public void setDesiredState(ChassisSpeeds desiredChassisSpeeds) {
    this.desiredChassisSpeeds = desiredChassisSpeeds;
  }

  public ChassisSpeeds getDesiredState() {
    return this.desiredChassisSpeeds;
  }

  public void setModulesToEncoders() {
    this.frontLeftModule.setModuleToEncoder();
    this.frontRightModule.setModuleToEncoder();
    this.backLeftModule.setModuleToEncoder();
    this.backRightModule.setModuleToEncoder();
  }

  @Override
  public void periodic() {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);

    SwerveModuleState frontLeftState = moduleStates[0];
    SwerveModuleState frontRightState = moduleStates[1];
    SwerveModuleState backLeftState = moduleStates[2];
    SwerveModuleState backRightState = moduleStates[3];

    publisher.set(new SwerveModuleState[] {
        frontLeftState,
        frontRightState,
        backLeftState,
        backRightState
    });

    this.frontLeftModule.setModuleState(frontLeftState);
    this.frontRightModule.setModuleState(frontRightState);
    this.backLeftModule.setModuleState(backLeftState);
    this.backRightModule.setModuleState(backRightState);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    this.periodic();
  }
}
