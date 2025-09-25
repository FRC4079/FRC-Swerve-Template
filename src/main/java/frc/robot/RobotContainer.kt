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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import xyz.malefic.frc.emu.Button.START
import xyz.malefic.frc.emu.Button.Y
import xyz.malefic.frc.pingu.Bingu.bindings

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    val pad: XboxController = XboxController(1)

    var networkChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser<Command?>("AutoChooser")

    /** The container for the robot. Contains subsystems, OI devices, and commands.  */
    init {
        val pad = XboxController(0)

        Swerve.defaultCommand = drive(pad)

        configureBindings()

        networkChooser.addDefaultOption("Do Nothing", PathPlannerAuto("Straight Auto"))
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * [Trigger] or our [JoystickButton] constructor with an arbitrary predicate, or via
     * the named factories in [CommandGenericHID]'s subclasses for [ ]/[CommandPS4Controller] controllers or [CommandJoystick].
     */
    private fun configureBindings() {
        pad.bindings {
            press(Y) { setTelePid() }
            press(START) { resetPidgey() }
        }
    }

    val autonomousCommand: Command?
        get() = networkChooser.get()
}
