package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final boolean OUTPUT_TO_SMART_DASH = true;

    private static final double WHEEL_RADIUS_METERS = 0.0508; // from the internet
    private static final double STEERING_GEAR_RATIO = 150.0/7.0; // from SDS website
    private static final double DRIVE_GEAR_RATIO = 6.75; // L2 from SDS website

    private final int steeringMotorCANId;

    private final TalonFX steeringMotor;
    private final TalonFX driveMotor;

    private final Canandmag encoder;
  
    public SwerveModule(int steeringMotorCANId, int driveMotorCANId, int encoderCANId) {
        this.steeringMotorCANId = steeringMotorCANId;

        this.steeringMotor = new TalonFX(steeringMotorCANId);
        this.driveMotor = new TalonFX(driveMotorCANId);
        this.encoder = new Canandmag(encoderCANId);

        var steeringConfigs = new TalonFXConfiguration();
        steeringConfigs.Slot0.kP = 0.16;
        steeringConfigs.Slot0.kI = 0.0;
        steeringConfigs.Slot0.kD = 0.0;
        steeringConfigs.Slot0.kV = 0.0; // feedforward term

        //experimenting with motion magic to smooth steering
       /*  var motionMagicConfigs = steeringConfigs.MotionMagic; 
        motionMagicConfigs.MotionMagicCruiseVelocity = 500; //Target cruise velocity of 500rps
        motionMagicConfigs.MotionMagicAcceleration = 1000; //Target acceleration of 1000rps/s (0.5 sec)
        motionMagicConfigs.MotionMagicJerk = 0; //Target jerk of 0 rps/s/s (0 sec)  */

        this.steeringMotor.getConfigurator().apply(steeringConfigs);

        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kP = 0.02;
        driveConfigs.Slot0.kI = 0.0;
        driveConfigs.Slot0.kD = 0.0;
        driveConfigs.Slot0.kV = 0.0; // feedforward term
        this.driveMotor.getConfigurator().apply(driveConfigs);
    }

    public void setModuleToEncoder() {
        // set the internal encoder based on the absolution position of the external encoder
        double encPosition = encoder.getAbsPosition(); // 0.0 to 1.0, inclusive, increasing counterclockwise
        steeringMotor.setPosition(STEERING_GEAR_RATIO*encPosition);

        // set the steering motor to the current position so it doesn't try to move
        double positionInRotations = STEERING_GEAR_RATIO*encPosition;
        ControlRequest steeringControlRequest = new PositionDutyCycle(positionInRotations);
        this.steeringMotor.setControl(steeringControlRequest);
    }

    public void setModuleState(SwerveModuleState state) {
        // get the current position of the steering motor and optimize the state
        // make sure positionOfSteering in [0.0, 1.0)
        double positionOfSteeringRad = 2*Math.PI*steeringMotor.getPosition().getValueAsDouble() / STEERING_GEAR_RATIO;
        state.optimize(new Rotation2d(-positionOfSteeringRad));

        // set the position of the steering motor
        // remember that angle is the negative of what the motors want, hence the minus
        double positionInRotations = -STEERING_GEAR_RATIO*state.angle.getDegrees()/360.0;
        ControlRequest steeringControlRequest = new PositionDutyCycle(positionInRotations);
        this.steeringMotor.setControl(steeringControlRequest);

        // calculate a speed scale factor (cosine compensation)
        double scaleFactor = state.angle.minus(new Rotation2d(-positionOfSteeringRad)).getCos();
        
        // set the speed of the drive motor
        final double FUDGE_FACTOR = 1.6; // TODO: Measure and change to EFFECTIVE_GEAR_RATIO, eliminating all the other constants here
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS_METERS;
        double wheelRotationsPerSecond = state.speedMetersPerSecond / wheelCircumference;
        double motorRotationsPerSecond = wheelRotationsPerSecond * DRIVE_GEAR_RATIO * FUDGE_FACTOR;
        ControlRequest driveControlRequest = new VelocityDutyCycle(motorRotationsPerSecond*scaleFactor);
        this.driveMotor.setControl(driveControlRequest);

        // this is probably not the best place for this code, but this is a sandbox project

        // if (OUTPUT_TO_SMART_DASH) {
        //     SmartDashboard.putNumber(
        //         "Swerve States/" + this.steeringMotorCANId + "/Drive/demand_wheelRotationsPerSecond", wheelRotationsPerSecond);
            
        //     SmartDashboard.putNumber(
        //         "Swerve States/" + this.steeringMotorCANId + "/Steering/demand_positionInRotations", positionInRotations);
        //     SmartDashboard.putNumber(
        //         "Swerve States/" + this.steeringMotorCANId + "/Steering/actual_positionOfSteering", positionOfSteeringRad);

        //     double encPosition = encoder.getAbsPosition(); // 0.0 to 1.0, inclusive, increasing counterclockwise
        //     SmartDashboard.putNumber(
        //         "Swerve States/" + this.steeringMotorCANId + "/external_encoderPosition", encPosition);        
        // }
    }
}
