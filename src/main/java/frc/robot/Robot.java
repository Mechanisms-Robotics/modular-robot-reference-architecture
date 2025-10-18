// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.time.Duration;
import java.time.Instant;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    private static final long SWERVE_ENDOCER_SET_FREQUECY_SECONDS = 1;
    public Instant lastSwerveModuleSetTime = Instant.MIN;

    private Command autonomousCommand;
    private final RobotContainer robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", "8736Framework"); // Set a metadata value

        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Publish data to NetworkTables
            Logger.addDataReceiver(new RLOGServer());
        } else {
            Logger.addDataReceiver(new RLOGServer());
        }

        Logger.start();
        SignalLogger.enableAutoLogging(false);

        this.robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // set the swerve modules to the external encoders periodically
        Instant now = Instant.now();
        Duration duration = Duration.between(this.lastSwerveModuleSetTime, now);
        if (
            duration.compareTo(
                Duration.ofSeconds(SWERVE_ENDOCER_SET_FREQUECY_SECONDS)
            ) >=
            0
        ) {
            this.robotContainer.setSwerveModulesToEncoders();
            this.lastSwerveModuleSetTime = now;
        }
    }

    @Override
    public void disabledExit() {
        // Initialize the pose estimator when we enable. This ensures the drivetrain
        // encoders have been set before we initialize. We will probably have to start
        // setting this differently at the start of autos.
        this.robotContainer.initializePoseEstimator();
    }

    @Override
    public void autonomousInit() {
        this.autonomousCommand = this.robotContainer.getAutonomousCommand();
        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
