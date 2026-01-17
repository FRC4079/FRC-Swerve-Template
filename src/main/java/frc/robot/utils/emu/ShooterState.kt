package frc.robot.utils.emu

enum class ShooterState(
    var velocity: Double,
) {
    OFF(0.0),
    FULL_SPEED(30.0),
    REVERSE(-15.0),
}

enum class HoodState(
    var velocity: Double,
) {
    /**
     * Should be unneeded when we implement auto-aim? | changes trajectory to be more upwards.
     */
    HOOD_UP(5.0),
    /**
     * Should be unneeded when we implement auto-aim? | changes trajectory to be more downwards.
     */
    HOOD_DOWN(-5.0),
    HOOD_NEUTRAL(0.0),
}