package frc.robot.commands

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.subsystems.Swerve
import frc.robot.utils.Direction
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds

/**
 * The [Kommand] object provides factory methods to create various commands
 * used in the robot's operation.
 *
 * This is called for instant commands instead of functions
 */
object Kommand {
    /**
     * Creates an [AlignSwerve] command to align the robot in a specified direction.
     *
     * @param dir The direction in which to align the robot.
     * @return An [AlignSwerve] command that aligns the robot.
     */
    @JvmStatic
    fun align(dir: Direction) = AlignSwerve(dir)

    /**
     * Creates a [PadDrive] command to control the robot's driving mechanism.
     *
     * @param controller The gaming controller used to drive the robot.
     * @param isFieldOriented Whether the driving should be field-oriented.
     * @return A [PadDrive] command to control the robot's driving mechanism.
     */
    @JvmStatic
    fun drive(controller: XboxController) = PadDrive(controller)

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
     * Creates a [PathPlannerAuto] command for autonomous operation.
     *
     * @return A [PathPlannerAuto] command for autonomous operation.
     */
    @JvmStatic
    fun autonomousCommand() = PathPlannerAuto(SwerveParameters.PATHPLANNER_AUTO_NAME)

    /**
     * Creates a [WaitCommand] to wait for a specified number of seconds.
     *
     * @param seconds The number of seconds to wait.
     * @return A [WaitCommand] that waits for the specified number of seconds.
     */
    @JvmStatic
    fun waitCmd(seconds: Double) = WaitCommand(seconds)
}
