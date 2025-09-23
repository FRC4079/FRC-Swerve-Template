package frc.robot.utils

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.PadDrive
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds
import frc.robot.utils.controller.GamingController

/**
 * The [Kommand] object provides factory methods to create various commands
 * used in the robot's operation.
 */
object Kommand {
    /**
     * Creates a [PadDrive] command to control the robot's driving mechanism.
     *
     * @param controller The gaming controller used to drive the robot.
     * @param isFieldOriented Whether the driving should be field-oriented.
     * @return A [PadDrive] command to control the robot's driving mechanism.
     */
    @JvmStatic
    fun drive(
        controller: GamingController,
        isFieldOriented: Boolean = Thresholds.IS_FIELD_ORIENTED,
    ) = PadDrive(controller, isFieldOriented)

    /**
     * Creates an [InstantCommand] to reset the Pidgey sensor.
     *
     * @return An [InstantCommand] that resets the Pidgey sensor.
     */
    @JvmStatic
    fun resetPidgey() = InstantCommand({ Swerve.resetPidgey() })

    /**
     * Creates an [InstantCommand] to set the teleoperation PID.
     *
     * @return An [InstantCommand] that sets the teleoperation PID.
     */
    @JvmStatic
    fun setTelePid() = InstantCommand({ Swerve.setTelePID() })

    /**
     * Creates a [WaitCommand] to wait for a specified number of seconds.
     *
     * @param seconds The number of seconds to wait.
     * @return A [WaitCommand] that waits for the specified number of seconds.
     */
    @JvmStatic
    fun waitCmd(seconds: Double) = WaitCommand(seconds)
}
