package frc.robot

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Kommand.drive
import frc.robot.commands.Kommand.resetPidgey
import frc.robot.commands.Kommand.setTelePid
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds
import frc.robot.utils.controller.GamingController
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import xyz.malefic.frc.emu.Button
import xyz.malefic.frc.emu.Button.A
import xyz.malefic.frc.emu.Button.B
import xyz.malefic.frc.emu.Button.X
import xyz.malefic.frc.emu.Button.Y
import xyz.malefic.frc.emu.Button.START
import xyz.malefic.frc.emu.Button.DPAD_UP
import xyz.malefic.frc.emu.Button.LEFT_BUMPER
import xyz.malefic.frc.emu.Button.LEFT_TRIGGER
import xyz.malefic.frc.emu.Button.RIGHT_BUMPER
import xyz.malefic.frc.emu.Button.RIGHT_STICK
import xyz.malefic.frc.pingu.Bingu.bindings

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {

    val pad: XboxController = XboxController(1)

    var networkChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("AutoChooser")

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {

        networkChooser.addDefaultOption("Do Nothing", PathPlannerAuto("Straight Auto"))

        configureBindings()
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [frc.robot.utils.controller.Trigger] or our [JoystickButton] constructor with an arbitrary predicate, or via
     * the named factories in [edu.wpi.first.wpilibj2.command.button.CommandGenericHID]'s subclasses for [edu.wpi.first.wpilibj2.command.button.CommandXboxController]/[edu.wpi.first.wpilibj2.command.button.CommandPS4Controller] controllers or [edu.wpi.first.wpilibj2.command.button.CommandJoystick].
     */
    private fun configureBindings() { // TODO: Remap bindings
        pad.bindings {
            press(Y) { setTelePid() }
            press(START) { resetPidgey() }
        }
    }

    val autonomousCommand: Command?
        get() = networkChooser.get()
}
