package frc.robot.utils.controller

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import frc.robot.utils.controller.Axis.LEFT_ANALOG_X
import frc.robot.utils.controller.Axis.LEFT_ANALOG_Y
import frc.robot.utils.controller.Axis.RIGHT_ANALOG_X
import frc.robot.utils.controller.Axis.RIGHT_ANALOG_Y
import frc.robot.utils.controller.Trigger.LEFT
import frc.robot.utils.controller.Trigger.RIGHT

/** A class representing a Logitech Gaming Pad.  */
class GamingController(
    usbPort: Int,
) : XboxController(usbPort) {
    private val gamepad: Joystick

    /**
     * Constructor for the LogitechGamingPad.
     *
     * @param usbPort The USB port the gamepad is connected to.
     */
    init {
        this.gamepad = Joystick(usbPort)
    }

    val leftTriggerValue: Double
        /**
         * Gets the value of the left trigger.
         *
         * @return The value of the left trigger.
         */
        get() = gamepad.getRawAxis(LEFT)

    val rightTriggerValue: Double
        /**
         * Gets the value of the right trigger.
         *
         * @return The value of the right trigger.
         */
        get() = gamepad.getRawAxis(RIGHT)

    val leftAnalogXAxis: Double
        /**
         * Gets the value of the left analog X axis.
         *
         * @return The value of the left analog X axis.
         */
        get() = gamepad.getRawAxis(LEFT_ANALOG_X)

    val leftAnalogYAxis: Double
        /**
         * Gets the value of the left analog Y axis.
         *
         * @return The value of the left analog Y axis.
         */
        get() = gamepad.getRawAxis(LEFT_ANALOG_Y)

    val rightAnalogXAxis: Double
        /**
         * Gets the value of the right analog X axis.
         *
         * @return The value of the right analog X axis.
         */
        get() = gamepad.getRawAxis(RIGHT_ANALOG_X)

    val rightAnalogYAxis: Double
        /**
         * Gets the value of the right analog Y axis.
         *
         * @return The value of the right analog Y axis.
         */
        get() = gamepad.getRawAxis(RIGHT_ANALOG_Y)

    override fun getAButton(): Boolean = gamepad.getRawButton(A)

    override fun getBButton(): Boolean = gamepad.getRawButton(B)

    override fun getXButton(): Boolean = gamepad.getRawButton(X)

    override fun getYButton(): Boolean = gamepad.getRawButton(Y)

    override fun getBackButton(): Boolean = gamepad.getRawButton(BACK)

    override fun getStartButton(): Boolean = gamepad.getRawButton(START)

    val dPadUp: Boolean
        get() = checkDPad(0)

    val dPadRight: Boolean
        get() = checkDPad(2)

    val dPadDown: Boolean
        get() = checkDPad(4)

    val dPadLeft: Boolean
        get() = checkDPad(6)

    /**
     * Checks if the DPad is pressed in a specific direction.
     *
     * @param index The index of the DPad direction.
     * @return True if the DPad is pressed in the specified direction, false otherwise.
     */
    fun checkDPad(index: Int): Boolean = 0 <= index && index <= 7 && (index * 45) == gamepad.getPOV()

    /**
     * Checks if the DPad is pressed at a specific angle.
     *
     * @param angle The angle to check.
     * @param inDegrees Whether the angle is in degrees.
     * @return True if the DPad is pressed at the specified angle, false otherwise.
     */
    fun checkDPad(
        angle: Double,
        inDegrees: Boolean,
    ): Boolean {
        val angdeg = if (inDegrees) angle else Math.toDegrees(angle)
        return angdeg.toInt() == gamepad.getPOV()
    }

    val dPad: Int
        /**
         * Gets the DPad value.
         *
         * @return The DPad value.
         */
        get() {
            val pov = gamepad.getPOV()
            return if (pov == -1) pov else pov / 45
        }

    /**
     * Gets the DPad value in degrees or radians.
     *
     * @param inDegrees Whether to return the value in degrees.
     * @return The DPad value in degrees or radians.
     */
    fun getDPad(inDegrees: Boolean): Double = if (inDegrees) gamepad.getPOV().toDouble() else Math.toRadians(gamepad.getPOV().toDouble())

    /**
     * Checks if the DPad is pressed.
     *
     * @return True if the DPad is pressed, false otherwise.
     */
    fun dPadIsPressed(): Boolean = gamepad.getPOV() != -1

    /**
     * Sets the rumble amount for the gamepad.
     *
     * @param amount The rumble amount.
     */
    fun setRumble(amount: Float) {
        gamepad.setRumble(RumbleType.kLeftRumble, amount.toDouble())
        gamepad.setRumble(RumbleType.kRightRumble, amount.toDouble())
    }

    override fun getRawAxis(which: Int): Double = gamepad.getRawAxis(which)

    override fun getRawButton(button: Int): Boolean = gamepad.getRawButton(button)
}
