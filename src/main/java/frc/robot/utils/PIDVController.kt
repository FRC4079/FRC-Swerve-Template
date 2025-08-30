package frc.robot.utils

import edu.wpi.first.math.controller.PIDController

/**
 * The PIDVController class extends the PIDController class to include an additional velocity term
 * (kV) for more precise control.
 */
class PIDVController : PIDController {
    /**
     * The velocity term (kV) used in the PIDVController.
     */
    var v: Double

    /**
     * Constructs a PIDVController with the specified PID constants and velocity term.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     * @param kV The velocity term.
     */
    constructor(kp: Double, ki: Double, kd: Double, kV: Double) : super(kp, ki, kd) {
        v = kV
    }

    /**
     * Constructs a PIDVController by copying the PID constants from an existing PIDController and
     * adding a velocity term.
     *
     * @param controller The existing PIDController to copy.
     * @param kV The velocity term.
     */
    constructor(controller: PIDController, kV: Double) : super(
        controller.p,
        controller.i,
        controller.d,
    ) {
        v = kV
    }
}
