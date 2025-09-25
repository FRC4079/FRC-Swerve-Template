package frc.robot.utils

/**
 * The Color enum represents possible default values that can be used for LED colors.
 */
enum class Color(val rgb: Triple<Int, Int, Int>) {
    /** Represents the color red.  */
    RED(Triple(255, 0, 0)),

    /** Represents the color green.  */
    GREEN(Triple(0, 255, 0)),

    /** Represents the color blue.  */
    BLUE(Triple(0, 0, 255)),

    /** Represents the color orange.  */
    ORANGE(Triple(255, 165, 0)),

    /** Represents the color purple.  */
    PURPLE(Triple(160, 32, 240)),

    /** Represents the color tan.  */
    TAN(Triple(255, 122, 20)),

    /** Represents the color high tide (a specific shade of blue-green).  */
    HIGH_TIDE(Triple(0, 200, 50)),
}