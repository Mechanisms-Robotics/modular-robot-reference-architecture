package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.controllers.DriveController;
import frc.robot.subsystems.drivetrain.Drive;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIORedux;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOSim;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final Drive drive;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(
        0
    );

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and a CANcoder
                drive = new Drive(
                    new GyroIORedux(),
                    new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                    new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                    new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                    new ModuleIOTalonFX(DriveConstants.BACK_RIGHT)
                );
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                    new GyroIO() {}, // Gryo is simulated via kinematics
                    new ModuleIOSim(DriveConstants.FRONT_LEFT),
                    new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                    new ModuleIOSim(DriveConstants.BACK_LEFT),
                    new ModuleIOSim(DriveConstants.BACK_RIGHT)
                );
                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                );
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>(
            "Auto Choices",
            AutoBuilder.buildAutoChooser()
        );

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveController.wheelRadiusCharacterization(drive)
        );
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveController.feedforwardCharacterization(drive)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(
            DriveController.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
            )
        );

        // Lock to 0° when A button is held
        controller
            .a()
            .whileTrue(
                DriveController.joystickDriveAtAngle(
                    drive,
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX(),
                    () -> Rotation2d.kZero
                )
            );

        // Switch to X pattern when X button is pressed
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        controller
            .b()
            .onTrue(
                Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                Rotation2d.kZero
                            )
                        ),
                    drive
                ).ignoringDisable(true)
            );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
