package frc.robot.subsystems.drivetrain;

import static frc.robot.CONSTANTS.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimModule implements SwerveModuleIO {

    private static final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor turnMotorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim driveSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            driveMotorModel,
            DRIVE_MOI,
            DRIVE_GEAR_RATIO
        ),
        driveMotorModel
    );
    private final DCMotorSim turnSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            turnMotorModel,
            TURN_MOI,
            STEERING_GEAR_RATIO
        ),
        turnMotorModel
    );

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    // PID gains tuned for simulation - adjust as needed
    private final PIDController driveController = new PIDController(0.05, 0, 0);
    private final PIDController turnController = new PIDController(8.0, 0, 0);
    private double driveFFVolts = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private Rotation2d turnAbsolutePosition = new Rotation2d();

    public SimModule() {
        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =
                driveFFVolts +
                driveController.calculate(
                    driveSim.getAngularVelocityRadPerSec()
                );
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(
                turnSim.getAngularPositionRad()
            );
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(
            MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)
        );
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(LOOP_PERIOD_SECS);
        turnSim.update(LOOP_PERIOD_SECS);

        // Update inputs
        inputs.data = new ModuleIOData(
            true, // driveConnected
            driveSim.getAngularPositionRad(),
            driveSim.getAngularVelocityRadPerSec(),
            driveAppliedVolts,
            Math.abs(driveSim.getCurrentDrawAmps()),
            0.0, // driveTorqueCurrentAmps (not available in sim)
            true, // turnConnected
            true, // turnEncoderConnected
            turnAbsolutePosition, // simulated absolute encoder
            new Rotation2d(turnSim.getAngularPositionRad()),
            turnSim.getAngularVelocityRadPerSec(),
            turnAppliedVolts,
            Math.abs(turnSim.getCurrentDrawAmps()),
            0.0 // turnTorqueCurrentAmps (not available in sim)
        );

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryDrivePositionsRad = new double[] {
            inputs.data.drivePositionRad(),
        };
        inputs.odometryTurnPositions = new Rotation2d[] {
            inputs.data.turnPosition(),
        };
    }

    @Override
    public void calibrate() {
        // In simulation, sync the absolute encoder to the relative position
        turnAbsolutePosition = new Rotation2d(turnSim.getAngularPositionRad());
    }

    @Override
    public void runDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void runTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveClosedLoop = true;
        driveFFVolts = feedforward;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void runTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveController.setPID(kP, kI, kD);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnController.setPID(kP, kI, kD);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        // Not applicable in simulation
    }
}
