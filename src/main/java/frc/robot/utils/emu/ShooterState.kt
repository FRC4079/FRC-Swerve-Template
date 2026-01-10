package frc.robot.utils.emu

enum class ShooterState(
    var velocity: Double,
) {
    OFF(0.0),
    HALF_SPEED(15.0),
    FULL_SPEED(30.0),
    REVERSE(15.0)
}