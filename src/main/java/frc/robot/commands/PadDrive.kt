package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Swerve
import frc.robot.utils.Dash.log
import frc.robot.utils.RobotParameters
import frc.robot.utils.RobotParameters.MotorParameters.MAX_SPEED
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE
import frc.robot.utils.controller.GamingController
import kotlin.math.abs

/** Command to control the robot's swerve drive using a Logitech gaming pad.  */
class PadDrive(
    private val pad: GamingController,
    private val isFieldOriented: Boolean,
) : Command() {
    /**
     * Constructs a new PadDrive command.
     *
     * @param pad The Logitech gaming pad used to control the robot.
     * @param isFieldOriented Whether the drive is field-oriented.
     */
    init {
        addRequirements(Swerve)
    }

    /**
     * Called every time the scheduler runs while the command is scheduled. This method retrieves the
     * current position from the gaming pad, calculates the rotation, logs the joystick values, and
     * sets the drive speeds for the swerve subsystem.
     */
    override fun execute() {
        val position: Pair<Double, Double> = positionSet(pad)

        val rotation =
            if (abs(pad.rightAnalogXAxis) >= 0.1) {
                -pad.rightAnalogXAxis * RobotParameters.MotorParameters.MAX_ANGULAR_SPEED
            } else {
                0.0
            }

        log("X Joystick", position.first)
        log("Y Joystick", position.second)
        log("Rotation", rotation)

        Swerve
            .setDriveSpeeds(position.second, position.first, rotation * 0.5, isFieldOriented)
    }

    /**
     * Returns true when the command should end.
     *
     * @return Always returns false, as this command never ends on its own.
     */
    override fun isFinished(): Boolean = false

    companion object {
        /**
         * Sets the position based on the input from the Logitech gaming pad.
         *
         * @param pad The Logitech gaming pad.
         * @return The coordinate representing the position. The first element is the x-coordinate, and
         * the second element is the y-coordinate.
         */
        fun positionSet(pad: GamingController): Pair<Double, Double> {
            var x: Double = -pad.leftAnalogXAxis * MAX_SPEED
            if (abs(x) < X_DEADZONE) x = 0.0

            var y: Double = -pad.leftAnalogYAxis * MAX_SPEED
            if (abs(y) < Y_DEADZONE) y = 0.0

            return x to y
        }
    }
}
