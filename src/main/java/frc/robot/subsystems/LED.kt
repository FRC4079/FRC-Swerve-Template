package frc.robot.subsystems

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.RobotState
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kotlin.math.sin

object LED : SubsystemBase() {
    private val alignmentIndication1 = AddressableLED(9)
    private val addressableLEDBuffer = AddressableLEDBuffer(120)

    /**
     * Creates a new instance of this LEDSubsystem. This constructor is private since this class is a
     * Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
    init {
        alignmentIndication1.setLength(addressableLEDBuffer.length)
        alignmentIndication1.setData(addressableLEDBuffer)
        alignmentIndication1.start()
    }

    /**
     * This method will be called once per scheduler run. Updates the LED pattern based on the robot
     * state.
     */
    override fun periodic() {
        if (RobotState.isDisabled()) {
            highTideFlow()
        }
    }

    /**
     * Sets the color for each of the LEDs based on RGB values.
     *
     * @param r (Red) Integer values between 0 - 255
     * @param g (Green) Integer values between 0 - 255
     * @param b (Blue) Integer values between 0 - 255
     */
    fun setRGB(
        r: Int,
        g: Int,
        b: Int,
    ) {
        for (i in 0..<addressableLEDBuffer.length) {
            addressableLEDBuffer.setRGB(i, r, g, b)
        }
        alignmentIndication1.setData(addressableLEDBuffer)
    }

    /**
     * Sets the color for each of the LEDs based on HSV values
     *
     * @param h (Hue) Integer values between 0 - 180
     * @param s (Saturation) Integer values between 0 - 255
     * @param v (Value) Integer values between 0 - 255
     */
    fun rainbowHSV(
        h: Int,
        s: Int,
        v: Int,
    ) {
        for (i in 0..<addressableLEDBuffer.length) {
            addressableLEDBuffer.setHSV(i, h, s, v)
        }
        alignmentIndication1.setData(addressableLEDBuffer)
    }

    /** Sets the LED color to tan.  */
    fun setTan() {
        setRGB(255, 122, 20)
    }

    /** Sets the LED color to red.  */
    fun setRed() {
        setRGB(255, 0, 0)
    }

    /** Sets the LED color to green.  */
    fun setGreen() {
        setRGB(0, 255, 0)
    }

    /**
     * Sets the LED color to orange. This is a specific shade of orange that is used for the LED
     * strip.
     */
    fun setOrange() {
        setRGB(255, 165, 0)
    }

    /** Sets the LED color to purple.  */
    fun setPurpleColor() {
        setRGB(160, 32, 240)
    }

    /** Sets the LED color to high tide (a specific shade of blue-green).  */
    fun setHighTide() {
        setRGB(0, 182, 174)
    }

    /**
     * Creates a flowing high tide effect on the LED strip. The effect is based on a sine wave pattern
     * that changes over time.
     */
    fun highTideFlow() {
        val currentTime = System.currentTimeMillis()
        val length = addressableLEDBuffer.length

        val waveSpeed = 30
        val waveWidth = 55

        for (i in 0..<length) {
            var wave = sin((i + (currentTime.toDouble() / waveSpeed)) % length * (2 * Math.PI / waveWidth))

            wave = (wave + 1) / 2

            val r = (wave * 0).toInt()
            val g = (wave * 200).toInt()
            val b = (wave * 50).toInt()

            addressableLEDBuffer.setRGB(i, r, g, b)
        }
        alignmentIndication1.setData(addressableLEDBuffer)
    }
}
