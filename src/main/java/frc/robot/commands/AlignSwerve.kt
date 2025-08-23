package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.PhotonVision
import frc.robot.subsystems.Swerve
import frc.robot.utils.Direction
import frc.robot.utils.RobotParameters.SwerveParameters

class AlignSwerve : Command {
    private var yaw = 0.0
    private var y = 0.0
    private var dist = 0.0
    private var rotationalController: PIDController? = null
    private var yController: PIDController? = null
    private var disController: PIDController? = null
    private val offset = 0.0 // double offset is the left/right offset from the april tag to make it properly align

    /**
     * Creates a new AlignSwerve using the Direction Enum.
     *
     * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
     * "center".
     */
    constructor(offsetSide: Direction) {
        when (offsetSide) {
            Direction.LEFT -> this.offset = SwerveParameters.AUTO_ALIGN_SWERVE_LEFT
            Direction.RIGHT -> this.offset = SwerveParameters.AUTO_ALIGN_SWERVE_RIGHT
            Direction.CENTER -> this.offset = 0.0
        }

        addRequirements(Swerve.getInstance())
    }

    /**
     * Creates a new AlignSwerve.
     *
     * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
     * "center".
     * @param offsetAmount The amount to offset the alignment by.
     */
    constructor(offsetSide: Direction, offsetAmount: Double) {
        // TODO: Placeholder for the offset amount, figure out the correct value
        when (offsetSide) {
            Direction.LEFT -> this.offset = -offsetAmount
            Direction.RIGHT -> this.offset = offsetAmount
            Direction.CENTER -> this.offset = 0.0
        }

        addRequirements(Swerve.getInstance())
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled.  */
    override fun initialize() {
        yaw = PhotonVision.getInstance().getYaw()
        y = PhotonVision.getInstance().getY()
        dist = PhotonVision.getInstance().getDist()

        rotationalController =
            PIDController(ROTATIONAL_PID.getP(), ROTATIONAL_PID.getI(), ROTATIONAL_PID.getD())
        // with the L4 branches
        val tolerance = 0.4
        rotationalController!!.setTolerance(tolerance)
        rotationalController!!.setSetpoint(0.0)

        yController = PIDController(Y_PID.getP(), Y_PID.getI(), Y_PID.getD())
        yController!!.setTolerance(1.5)
        yController!!.setSetpoint(0.0)

        disController = PIDController(DIST_PID.getP(), DIST_PID.getI(), DIST_PID.getD())
        disController!!.setTolerance(1.5)
        disController!!.setSetpoint(0.0)
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until [.isFinished]) returns true.)
     */
    override fun execute() {
        yaw = PhotonVision.getInstance().getYaw()
        y = PhotonVision.getInstance().getY()
        dist = PhotonVision.getInstance().getDist()

        Swerve.getInstance()
            .setDriveSpeeds(
                disController!!.calculate(dist),
                yController!!.calculate(y) + offset,
                rotationalController!!.calculate(yaw),
                false
            )
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this method
     * returning true -- the scheduler will call its [.end] method.
     *
     *
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always return
     * true will result in the command executing once and finishing immediately. It is recommended to
     * use * [InstantCommand][edu.wpi.first.wpilibj2.command.InstantCommand] for such an
     * operation.
     *
     * @return whether this command has finished.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is called when [.isFinished] returns true -- or when it is interrupted/canceled.
     * This is where you may want to wrap up loose ends, like shutting off a motor that was being used
     * in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    override fun end(interrupted: Boolean) {
        Swerve.getInstance().stop()
    }
}
