// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Mode lifecycle for the robot. Keep logic out of here — this class only
 * wires up AdvantageKit logging, owns the dashboard-level choosers, and
 * schedules/cancels the autonomous command at mode transitions. Everything
 * robot-specific lives in RobotContainer and the subsystems.
 *
 * <p>Extends LoggedRobot (AdvantageKit): every input the code reads goes
 * through Logger.processInputs, which is what makes log replay possible.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  private final SendableChooser<Boolean> resetPoseChooser = new SendableChooser<>();
  private boolean lastResetPoseSelected = false;

  public Robot() {
    // Phoenix's own hoot logger would double-log every CAN signal next to
    // AdvantageKit; keep it off.
    SignalLogger.enableAutoLogging(false);

    switch (CONSTANTS.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        //Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
          new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))
        );
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    this.robotContainer = new RobotContainer();

    resetPoseChooser.setDefaultOption("None", false);
    resetPoseChooser.addOption("All", true);
    SmartDashboard.putData("Reset Pose", resetPoseChooser);

    // Placeholder until the drive team picks an auto while disabled; never
    // null so mode transitions don't have to null-check.
    this.autonomousCommand = Commands.none();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Force the auto selection back to "None" on boot, even if the dashboard
    // (Elastic) remembered a selection from last time. Prevents the robot
    // from running an auto nobody intentionally selected this power-on.
    SmartDashboard.putString("Auto Chooser/selected", "None");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Dashboard "Reset Pose" control. Fire ONCE when the selection flips to
    // "All" — the previous code reset every loop while "All" stayed selected,
    // which continuously cleared the pose estimator's latency-compensation
    // buffer and made it silently drop all vision measurements.
    boolean resetPoseSelected = Boolean.TRUE.equals(
        resetPoseChooser.getSelected());
    if (resetPoseSelected && !lastResetPoseSelected) {
      robotContainer.drivetrain.resetPose(robotContainer.drivetrain.getPose());
    }
    lastResetPoseSelected = resetPoseSelected;
  }

  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {
    // Rebuild the autonomous command whenever the drive team picks a
    // different auto on the dashboard. Building it while disabled (instead of
    // in autonomousInit) means the command is ready the instant auto starts.
    String selectedAuto = this.robotContainer.autoChooser.getSelected();
    if (selectedAuto != null
        && !selectedAuto.equals(this.autonomousCommand.getName())) {
      this.autonomousCommand =
          this.robotContainer.getAutonomousCommand(selectedAuto);
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (this.autonomousCommand != null) {
      // Command.schedule() is deprecated for removal in 2026; scheduling via
      // the CommandScheduler is the supported path.
      CommandScheduler.getInstance().schedule(this.autonomousCommand);
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
