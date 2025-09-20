package frc.robot

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Kommand.drive
import frc.robot.commands.Kommand.resetPidgey
import frc.robot.commands.Kommand.setTelePid
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds
import frc.robot.utils.controller.GamingController
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    private val padA: JoystickButton
    private val padB: JoystickButton
    private val padX: JoystickButton
    private val padY: JoystickButton
    private val padStart: JoystickButton
    private val padLeftBumper: JoystickButton
    private val padRightBumper: JoystickButton

    var networkChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("AutoChooser")

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        val pad = GamingController(0)
        padStart = JoystickButton(pad, 8)
        padA = JoystickButton(pad, 1)
        padB = JoystickButton(pad, 2)
        padX = JoystickButton(pad, 3)
        padY = JoystickButton(pad, 4)
        padLeftBumper = JoystickButton(pad, 5)
        padRightBumper = JoystickButton(pad, 6)

        Swerve.defaultCommand = drive(pad, Thresholds.IS_FIELD_ORIENTED)

        configureBindings()

        networkChooser.addDefaultOption("Do Nothing", PathPlannerAuto("Straight Auto"))
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [frc.robot.utils.controller.Trigger] or our [JoystickButton] constructor with an arbitrary predicate, or via
     * the named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for [edu.wpi.first.wpilibj2.command.button.CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() { // TODO: Remap bindings
        padStart.onTrue(resetPidgey())
        padY.onTrue(setTelePid())
    }

    val autonomousCommand: Command?
        get() = networkChooser.get()
}