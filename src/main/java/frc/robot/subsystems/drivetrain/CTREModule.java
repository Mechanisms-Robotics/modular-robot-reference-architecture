package frc.robot.subsystems.drivetrain;

import static frc.robot.CONSTANTS.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class CTREModule implements SwerveModuleIO {

    // Hardware objects
    private final TalonFX turnMotor;
    private final TalonFX driveMotor;
    private final Canandmag encoder;

    // Config objects
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private static final Executor brakeModeExecutor =
        Executors.newFixedThreadPool(8);

    // Control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(
        0
    ).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC positionTorqueRequest =
        new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest =
        new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

    // Status signals for turn motor
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnSupplyCurrentAmps;
    private final StatusSignal<Current> turnTorqueCurrentAmps;

    // Status signals for drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrentAmps;
    private final StatusSignal<Current> driveTorqueCurrentAmps;

    public CTREModule(
        int turnMotorCANId,
        int driveMotorCANId,
        int encoderCANId
    ) {
        this.turnMotor = new TalonFX(turnMotorCANId);
        this.driveMotor = new TalonFX(driveMotorCANId);
        this.encoder = new Canandmag(encoderCANId);

        // Configure turn motor
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0.kP = 0.16;
        turnConfig.Slot0.kI = 0.0;
        turnConfig.Slot0.kD = 0.0;
        turnConfig.Feedback.SensorToMechanismRatio = STEERING_GEAR_RATIO;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            TURN_CURRENT_LIMIT_AMPS;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            -TURN_CURRENT_LIMIT_AMPS;
        turnConfig.CurrentLimits.StatorCurrentLimit = TURN_CURRENT_LIMIT_AMPS;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        this.turnMotor.getConfigurator().apply(turnConfig);

        // Configure drive motor
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = 0.02;
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = 0.0;
        driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
            DRIVE_CURRENT_LIMIT_AMPS;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
            -DRIVE_CURRENT_LIMIT_AMPS;
        driveConfig.CurrentLimits.StatorCurrentLimit = DRIVE_CURRENT_LIMIT_AMPS;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        this.driveMotor.getConfigurator().apply(driveConfig);

        // Create turn status signals
        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnSupplyCurrentAmps = turnMotor.getSupplyCurrent();
        turnTorqueCurrentAmps = turnMotor.getTorqueCurrent();

        // Create drive status signals
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
        driveTorqueCurrentAmps = driveMotor.getTorqueCurrent();

        // Configure update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps,
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps
        );

        // Optimize bus utilization
        ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        BaseStatusSignal.refreshAll(
            turnPosition,
            turnVelocity,
            turnAppliedVolts,
            turnSupplyCurrentAmps,
            turnTorqueCurrentAmps,
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrentAmps,
            driveTorqueCurrentAmps
        );

        // Update drive inputs
        inputs.data = new ModuleIOData(
            BaseStatusSignal.isAllGood(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrentAmps,
                driveTorqueCurrentAmps
            ),
            Units.rotationsToRadians(drivePosition.getValueAsDouble()),
            Units.rotationsToRadians(driveVelocity.getValueAsDouble()),
            driveAppliedVolts.getValueAsDouble(),
            driveSupplyCurrentAmps.getValueAsDouble(),
            driveTorqueCurrentAmps.getValueAsDouble(),
            BaseStatusSignal.isAllGood(
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrentAmps,
                turnTorqueCurrentAmps
            ),
            true, // Canandmag encoder always connected (no built-in status signal)
            Rotation2d.fromRotations(encoder.getAbsPosition()),
            Rotation2d.fromRotations(turnPosition.getValueAsDouble()),
            Units.rotationsToRadians(turnVelocity.getValueAsDouble()),
            turnAppliedVolts.getValueAsDouble(),
            turnSupplyCurrentAmps.getValueAsDouble(),
            turnTorqueCurrentAmps.getValueAsDouble()
        );
    }

    @Override
    public void calibrate() {
        // Set the turn motor position to match the absolute encoder
        // This calibrates the relative encoder to the absolute position
        double absolutePositionRotations = encoder.getAbsPosition();
        double motorPositionRotations =
            absolutePositionRotations * STEERING_GEAR_RATIO;
        turnMotor.setPosition(motorPositionRotations);
    }

    @Override
    public void runDriveOpenLoop(double output) {
        driveMotor.setControl(torqueCurrentRequest.withOutput(output));
    }

    @Override
    public void runTurnOpenLoop(double output) {
        turnMotor.setControl(torqueCurrentRequest.withOutput(output));
    }

    @Override
    public void runDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveMotor.setControl(
            velocityTorqueRequest
                .withVelocity(Units.radiansToRotations(velocityRadPerSec))
                .withFeedForward(feedforward)
        );
    }

    @Override
    public void runTurnPosition(Rotation2d rotation) {
        turnMotor.setControl(
            this.positionTorqueRequest.withPosition(rotation.getRotations())
        );
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveConfig.Slot0.kP = kP;
        driveConfig.Slot0.kI = kI;
        driveConfig.Slot0.kD = kD;
        driveMotor.getConfigurator().apply(driveConfig, 0.25);
    }

    @Override
    public void setTurnPID(double kP, double kI, double kD) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kI = kI;
        turnConfig.Slot0.kD = kD;
        turnMotor.getConfigurator().apply(turnConfig, 0.25);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (driveConfig) {
                driveConfig.MotorOutput.NeutralMode = enabled
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast;
                driveMotor.getConfigurator().apply(driveConfig, 0.25);
            }
        });
        brakeModeExecutor.execute(() -> {
            synchronized (turnConfig) {
                turnConfig.MotorOutput.NeutralMode = enabled
                    ? NeutralModeValue.Brake
                    : NeutralModeValue.Coast;
                turnMotor.getConfigurator().apply(turnConfig, 0.25);
            }
        });
    }
}
