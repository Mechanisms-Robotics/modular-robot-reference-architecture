package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
    private final CommandPS5Controller controller = new CommandPS5Controller(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and a CANcoder
                this.drive = new Drive(
                    new GyroIORedux(),
                    new ModuleIOTalonFX(DriveConstants.FRONT_LEFT),
                    new ModuleIOTalonFX(DriveConstants.FRONT_RIGHT),
                    new ModuleIOTalonFX(DriveConstants.BACK_LEFT),
                    new ModuleIOTalonFX(DriveConstants.BACK_RIGHT)
                );
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                this.drive = new Drive(
                    new GyroIO() {}, // Gryo is simulated via kinematics
                    new ModuleIOSim(DriveConstants.FRONT_LEFT),
                    new ModuleIOSim(DriveConstants.FRONT_RIGHT),
                    new ModuleIOSim(DriveConstants.BACK_LEFT),
                    new ModuleIOSim(DriveConstants.BACK_RIGHT)
                );
                break;
            default:
                // Replayed robot, disable IO implementations
                this.drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                );
                break;
        }

        // Set up auto routines
        this.autoChooser = new LoggedDashboardChooser<>(
            "Auto Choices",
            AutoBuilder.buildAutoChooser()
        );

        // Set up SysId routines
        this.autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveController.wheelRadiusCharacterization(this.drive)
        );
        this.autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveController.feedforwardCharacterization(this.drive)
        );
        this.autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            this.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        );
        this.autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            this.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        );
        this.autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            this.drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        );
        this.autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            this.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
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
        this.drive.setDefaultCommand(
            DriveController.joystickDrive(
                this.drive,
                () -> -this.controller.getLeftY(),
                () -> -this.controller.getLeftX(),
                () -> -this.controller.getRightX()
            )
        );

        // Lock to 0 degrees when A button is held
        this.controller.circle().whileTrue(
            DriveController.joystickDriveAtAngle(
                this.drive,
                () -> -this.controller.getLeftY(),
                () -> -this.controller.getLeftX(),
                () -> Rotation2d.kZero
            )
        );

        // Switch to X pattern when X button is pressed
        this.controller.square().onTrue(
            Commands.runOnce(this.drive::stopWithX, this.drive)
        );

        // Reset gyro to 0 degrees when B button is pressed
        this.controller.cross().onTrue(
            Commands.runOnce(
                () ->
                    this.drive.setPose(
                        new Pose2d(
                            this.drive.getPose().getTranslation(),
                            Rotation2d.kZero
                        )
                    ),
                this.drive
            ).ignoringDisable(true)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }
}
