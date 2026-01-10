package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.Kommand.drive
import frc.robot.commands.Kommand.resetPidgey
import frc.robot.commands.Kommand.setTelePid
import frc.robot.subsystems.LED
import frc.robot.subsystems.Swerve
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import xyz.malefic.frc.emu.Button.START
import xyz.malefic.frc.emu.Button.Y
import xyz.malefic.frc.pingu.binding.Bingu.bindings

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
    val pad: XboxController = XboxController(1)
    var networkChooser: LoggedDashboardChooser<Command?> = LoggedDashboardChooser("AutoChooser")

    init {
        CommandScheduler.getInstance().registerSubsystem(
            Swerve,
            LED,
//            PhotonVision,
        )

        val pad = XboxController(0)

        Swerve.defaultCommand = drive(pad)

        configureBindings()

//        networkChooser.addDefaultOption("Do Nothing", PathPlannerAuto("Straight Auto"))
    }

    /**
     * This method defines bindings, i.e., which buttons trigger which commands.
     */
    private fun configureBindings() {
        pad.bindings {
            press(Y) { CommandScheduler.getInstance().schedule(setTelePid()) }
            press(START) { CommandScheduler.getInstance().schedule(resetPidgey()) }
        }
    }

    val autonomousCommand: Command?
        get() = networkChooser.get()
}
